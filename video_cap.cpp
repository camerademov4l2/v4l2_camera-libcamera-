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
#include <vector>
#include <map>
#include <filesystem>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

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

// YUYV to RGB conversion function
/*void YUYVToRGB(const uint8_t* yuyv, uint8_t* rgb, int width, int height) {
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
}*/

void YUYVToRGB(const uint8_t* yuyv, uint8_t* rgb, int width, int height) {
    cv::Mat yuv_frame(height, width, CV_8UC2, const_cast<uint8_t*>(yuyv));
    cv::Mat rgb_frame(height, width, CV_8UC3);
    cv::cvtColor(yuv_frame, rgb_frame, cv::COLOR_YUV2BGR_YUYV);
    std::memcpy(rgb, rgb_frame.data, width * height * 3);
}

class VideoRecordingHandler {
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
    cv::VideoWriter videoWriter_;
    int recordingDuration_ = 10; // 5 seconds

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
                    
                    // Append frame to video
                    cv::Mat frame(height, width, CV_8UC3, rgb_buffer.data());
                    videoWriter_.write(frame);
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
    VideoRecordingHandler() 
        : allocator_(nullptr)
        , captureComplete_(false)
        , cameraRunning_(false) {
        cameraManager_ = std::make_unique<libcamera::CameraManager>();
    }
    
    ~VideoRecordingHandler() {
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
        
        // Configure for video capture
        std::vector<libcamera::StreamRole> roles = {libcamera::StreamRole::VideoRecording};
        config_ = camera_->generateConfiguration(roles);
        if (!config_) {
            std::cerr << "Failed to generate configuration" << std::endl;
            return false;
        }
        
        // Configure stream with improved settings
        libcamera::StreamConfiguration &streamConfig = config_->at(0);
        streamConfig.pixelFormat = libcamera::formats::YUYV;
        streamConfig.size.width = 1280;
        streamConfig.size.height = 720;
        streamConfig.bufferCount = 4; // Increase buffer count for video
        
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
        
        // Create video writer
        fs::path videoPath = "captured_video.mp4";
        int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        videoWriter_ = cv::VideoWriter(videoPath.string(), fourcc, 30.0, 
                                      cv::Size(config_->at(0).size.width, 
                                               config_->at(0).size.height));
        
        return true;
    }
    
    bool captureVideo() {
    captureComplete_ = false;
    
    auto start = std::chrono::steady_clock::now();
    auto end = start + std::chrono::seconds(recordingDuration_);
    
    while (std::chrono::steady_clock::now() < end) {
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
        camera_->requestCompleted.connect(this, &VideoRecordingHandler::requestComplete);
        
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
            if (!completionCondition_.wait_for(lock, std::chrono::seconds(1),
                [this] { return captureComplete_ || quit.load(); })) {
                std::cerr << "Capture timeout" << std::endl;
                return false;
            }
        }
        
        stopCamera();
        requests_.clear();
        captureComplete_ = false;
    }
    
    // Release video writer
    videoWriter_.release();
    return true;
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
            camera_->requestCompleted.disconnect(this, &VideoRecordingHandler::requestComplete);
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
static std::unique_ptr<VideoRecordingHandler> globalCaptureHandler;

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
        globalCaptureHandler = std::make_unique<VideoRecordingHandler>();
        
        if (!globalCaptureHandler->initialize()) {
            std::cerr << "Failed to initialize camera" << std::endl;
            return -1;
        }
        
        // Wait for sensor to settle and auto-exposure to stabilize
        std::cout << "Waiting for sensor to initialize..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Capture video
        std::cout << "Capturing video..." << std::endl;
        if (!globalCaptureHandler->captureVideo()) {
            std::cerr << "Failed to capture video" << std::endl;
            return -1;
        }
        
        // Allow time for video saving to complete
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
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
