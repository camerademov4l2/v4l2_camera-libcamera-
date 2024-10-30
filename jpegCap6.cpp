#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <libcamera/libcamera.h>
#include <memory>
#include <signal.h>
#include <fstream>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <sys/mman.h>
#include <jpeglib.h>
#include <cstring>
#include <cerrno>
#include <vector>
#include <map>

using namespace std::chrono_literals;

// Global flag for signal handling
std::atomic<bool> quit(false);

// Color conversion constants
constexpr int Y_OFFSET = 16;
constexpr int UV_OFFSET = 128;
constexpr int Y_SCALE = 298;
constexpr int RED_V_SCALE = 409;
constexpr int GREEN_U_SCALE = -100;
constexpr int GREEN_V_SCALE = -208;
constexpr int BLUE_U_SCALE = 516;

// Utility function to clamp values between 0 and 255
inline uint8_t clamp(int value) {
    return static_cast<uint8_t>(std::clamp(value, 0, 255));
}

// Forward declarations
class StillCaptureHandler;
void signalHandler(int signum);

// YUYV to RGB conversion function
void YUYVToRGB(const uint8_t* yuyv, uint8_t* rgb, int width, int height) {
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j += 2) {
            int index = (i * width + j) * 2;
            int rgb_index = (i * width + j) * 3;
            
            int y0 = yuyv[index + 0] - Y_OFFSET;
            int u = yuyv[index + 1] - UV_OFFSET;
            int y1 = yuyv[index + 2] - Y_OFFSET;
            int v = yuyv[index + 3] - UV_OFFSET;
            
            // First pixel
            int r = (Y_SCALE * y0 + RED_V_SCALE * v + 128) >> 8;
            int g = (Y_SCALE * y0 + GREEN_U_SCALE * u + GREEN_V_SCALE * v + 128) >> 8;
            int b = (Y_SCALE * y0 + BLUE_U_SCALE * u + 128) >> 8;
            
            rgb[rgb_index + 0] = clamp(r);
            rgb[rgb_index + 1] = clamp(g);
            rgb[rgb_index + 2] = clamp(b);
            
            // Second pixel
            r = (Y_SCALE * y1 + RED_V_SCALE * v + 128) >> 8;
            g = (Y_SCALE * y1 + GREEN_U_SCALE * u + GREEN_V_SCALE * v + 128) >> 8;
            b = (Y_SCALE * y1 + BLUE_U_SCALE * u + 128) >> 8;
            
            rgb[rgb_index + 3] = clamp(r);
            rgb[rgb_index + 4] = clamp(g);
            rgb[rgb_index + 5] = clamp(b);
        }
    }
}

// JPEG saving function
bool saveJPEG(const char* filename, uint8_t* rgb_buffer, int width, int height, int quality) {
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    
    FILE* outfile = fopen(filename, "wb");
    if (!outfile) {
        std::cerr << "Error: Cannot open output file " << filename << std::endl;
        return false;
    }
    
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_stdio_dest(&cinfo, outfile);
    
    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;
    
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);
    jpeg_start_compress(&cinfo, TRUE);
    
    std::vector<uint8_t> row_buffer(width * 3);
    JSAMPROW row_pointer[1];
    
    while (cinfo.next_scanline < cinfo.image_height) {
        std::memcpy(row_buffer.data(), 
               rgb_buffer + (cinfo.next_scanline * width * 3), 
               width * 3);
        row_pointer[0] = row_buffer.data();
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }
    
    jpeg_finish_compress(&cinfo);
    fclose(outfile);
    jpeg_destroy_compress(&cinfo);
    
    return true;
}


class StillCaptureHandler {
private:
    std::unique_ptr<libcamera::CameraManager> cameraManager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    libcamera::FrameBufferAllocator *allocator_;
    std::mutex completionMutex_;
    std::condition_variable completionCondition_;
    bool captureComplete_;
    std::atomic<bool> cameraRunning_;
    std::map<libcamera::FrameBuffer *, void *> mappedBuffers_;

    void* mapBuffer(libcamera::FrameBuffer* buffer) {
        if (!buffer || buffer->planes().empty()) {
            return nullptr;
        }
        
        const auto& plane = buffer->planes()[0];
        void* memory = mmap(nullptr, 
                          plane.length,
                          PROT_READ | PROT_WRITE,
                          MAP_SHARED,
                          plane.fd.get(),
                          0);
                          
        if (memory == MAP_FAILED) {
            std::cerr << "Failed to mmap buffer: " << std::strerror(errno) << std::endl;
            return nullptr;
        }
        
        return memory;
    }

    void unmapBuffer(libcamera::FrameBuffer* buffer, void* memory) {
        if (buffer && memory && !buffer->planes().empty()) {
            munmap(memory, buffer->planes()[0].length);
        }
    }
    
    void requestComplete(libcamera::Request *request) {
        if (request->status() == libcamera::Request::RequestComplete) {
            const libcamera::Request::BufferMap &buffers = request->buffers();
            for (auto &bufferPair : buffers) {
                libcamera::FrameBuffer *buffer = bufferPair.second;
                const libcamera::Stream *stream = bufferPair.first;
                
                void* memory = mappedBuffers_[buffer];
                
                if (memory) {
                    int width = stream->configuration().size.width;
                    int height = stream->configuration().size.height;
                    
                    // Allocate RGB buffer
                    std::vector<uint8_t> rgb_buffer(width * height * 3);
                    
                    // Convert YUYV to RGB with improved color conversion
                    YUYVToRGB(static_cast<uint8_t*>(memory), rgb_buffer.data(), width, height);
                    
                    // Save as JPEG with improved quality
                    if (saveJPEG("captured_image.jpg", rgb_buffer.data(), width, height, 95)) {
                        std::cout << "Image captured successfully - " << width << "x" << height << std::endl;
                    } else {
                        std::cerr << "Failed to save JPEG" << std::endl;
                    }
                }
            }
        } else {
            std::cerr << "Request failed with status: " << request->status() << std::endl;
        }
        
        std::unique_lock<std::mutex> lock(completionMutex_);
        captureComplete_ = true;
        completionCondition_.notify_one();
    }

public:
    StillCaptureHandler() 
        : allocator_(nullptr)
        , captureComplete_(false)
        , cameraRunning_(false) {
        cameraManager_ = std::make_unique<libcamera::CameraManager>();
    }
    
    ~StillCaptureHandler() {
        stopCamera();
        std::this_thread::sleep_for(100ms);
        cleanup();
    }
    
    bool initialize() {
        if (cameraManager_->start()) {
            std::cerr << "Failed to start camera manager" << std::endl;
            return false;
        }
        
        if (cameraManager_->cameras().empty()) {
            std::cerr << "No cameras found" << std::endl;
            return false;
        }
        
        camera_ = cameraManager_->cameras()[0];
        
        if (camera_->acquire()) {
            std::cerr << "Failed to acquire camera" << std::endl;
            return false;
        }
        
        // Configure for still capture
        std::vector<libcamera::StreamRole> roles = {libcamera::StreamRole::StillCapture};
        config_ = camera_->generateConfiguration(roles);
        if (!config_) {
            std::cerr << "Failed to generate configuration" << std::endl;
            return false;
        }
        
        // Configure stream with improved settings
        libcamera::StreamConfiguration &streamConfig = config_->at(0);
        streamConfig.pixelFormat = libcamera::formats::YUYV;
        streamConfig.size.width = 1920;  // Reduced resolution for testing
        streamConfig.size.height = 1080;
        streamConfig.bufferCount = 1;
        
        // Set additional controls for better image quality
        libcamera::ControlList controls;
        controls.set(libcamera::controls::AeExposureMode, libcamera::controls::ExposureNormal);
        controls.set(libcamera::controls::AwbMode, libcamera::controls::AwbAuto);
        
        if (camera_->configure(config_.get()) < 0) {
            std::cerr << "Failed to configure camera" << std::endl;
            return false;
        }
        
        // Allocate and map buffers
        allocator_ = new libcamera::FrameBufferAllocator(camera_);
        for (libcamera::StreamConfiguration &cfg : *config_) {
            if (allocator_->allocate(cfg.stream()) < 0) {
                std::cerr << "Failed to allocate buffers" << std::endl;
                return false;
            }

            const std::vector<std::unique_ptr<libcamera::FrameBuffer>> &buffers = 
                allocator_->buffers(cfg.stream());
            
            for (const std::unique_ptr<libcamera::FrameBuffer> &buffer : buffers) {
                void* memory = mapBuffer(buffer.get());
                if (!memory) {
                    std::cerr << "Failed to map buffer" << std::endl;
                    return false;
                }
                mappedBuffers_[buffer.get()] = memory;
            }
        }
        
        return true;
    }
    
    bool captureImage() {
        captureComplete_ = false;
        
        // Create and configure request
        std::unique_ptr<libcamera::Request> request = camera_->createRequest();
        if (!request) {
            std::cerr << "Failed to create request" << std::endl;
            return false;
        }
        
        const libcamera::Stream *stream = config_->at(0).stream();
        // Use const_cast since the API requires non-const Stream*
        const std::vector<std::unique_ptr<libcamera::FrameBuffer>> &buffers = 
            allocator_->buffers(const_cast<libcamera::Stream*>(stream));
        
        if (request->addBuffer(stream, buffers[0].get()) < 0) {
            std::cerr << "Failed to add buffer to request" << std::endl;
            return false;
        }
        
        requests_.push_back(std::move(request));
        
        // Connect signal and start camera
        camera_->requestCompleted.connect(this, &StillCaptureHandler::requestComplete);
        
        if (camera_->start() < 0) {
            std::cerr << "Failed to start camera" << std::endl;
            return false;
        }
        cameraRunning_ = true;
        
        // Queue capture request
        if (camera_->queueRequest(requests_[0].get()) < 0) {
            std::cerr << "Failed to queue request" << std::endl;
            return false;
        }
        
        // Wait for completion
        {
            std::unique_lock<std::mutex> lock(completionMutex_);
            if (!completionCondition_.wait_for(lock, std::chrono::seconds(5),
                [this] { return captureComplete_ || quit.load(); })) {
                std::cerr << "Capture timeout" << std::endl;
                return false;
            }
        }
        
        stopCamera();
        return captureComplete_;
    }
    
    void stopCamera() {
        if (cameraRunning_) {
            camera_->stop();
            std::this_thread::sleep_for(50ms);
            cameraRunning_ = false;
        }
    }
    
    bool isRunning() const {
        return cameraRunning_;
    }
    
private:
    void cleanup() {
        if (camera_) {
            stopCamera();
            camera_->requestCompleted.disconnect(this, &StillCaptureHandler::requestComplete);
            std::this_thread::sleep_for(100ms);
            
            requests_.clear();
            
            for (auto &pair : mappedBuffers_) {
                unmapBuffer(pair.first, pair.second);
            }
            mappedBuffers_.clear();
            
            delete allocator_;
            allocator_ = nullptr;
            
            std::this_thread::sleep_for(100ms);
            camera_->release();
            camera_.reset();
        }
        
        if (cameraManager_) {
            std::this_thread::sleep_for(100ms);
            cameraManager_->stop();
        }
    }
};

// Global capture handler declaration
static std::unique_ptr<StillCaptureHandler> globalCaptureHandler;

// Signal handler implementation
void signalHandler(int signum) {
    std::cout << "\nReceived signal " << signum << std::endl;
    quit.store(true);
    if (globalCaptureHandler && globalCaptureHandler->isRunning()) {
        globalCaptureHandler->stopCamera();
    }
}

// Main function
int main() {
    try {
        // Set up signal handling
        signal(SIGINT, signalHandler);
        signal(SIGTERM, signalHandler);
        
        // Create and initialize capture handler
        globalCaptureHandler = std::make_unique<StillCaptureHandler>();
        
        if (!globalCaptureHandler->initialize()) {
            std::cerr << "Failed to initialize camera" << std::endl;
            return -1;
        }
        
        // Wait for sensor to settle and auto-exposure to stabilize
        std::cout << "Waiting for sensor to initialize..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Capture image
        std::cout << "Capturing image..." << std::endl;
        if (!globalCaptureHandler->captureImage()) {
            std::cerr << "Failed to capture image" << std::endl;
            return -1;
        }
        
        // Allow time for image saving to complete
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Clean shutdown
        std::cout << "Shutting down camera..." << std::endl;
        globalCaptureHandler->stopCamera();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Reset handler which will trigger cleanup
        globalCaptureHandler.reset();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        std::cout << "Camera shutdown complete" << std::endl;
        
    } catch (const std::exception &e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        
        // Attempt cleanup even after error
        if (globalCaptureHandler) {
            globalCaptureHandler->stopCamera();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            globalCaptureHandler.reset();
        }
        return -1;
    }
    
    return 0;
}
