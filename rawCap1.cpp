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

using namespace std::chrono_literals;

// Global flag for signal handling
std::atomic<bool> quit(false);

// Forward declaration
void signalHandler(int signum);

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
        void* memory = nullptr;
        const std::vector<libcamera::FrameBuffer::Plane>& planes = buffer->planes();
        
        if (!planes.empty()) {
            memory = mmap(nullptr, 
                         planes[0].length,
                         PROT_READ | PROT_WRITE,
                         MAP_SHARED,
                         planes[0].fd.get(),
                         0);
            
            if (memory == MAP_FAILED) {
                std::cerr << "Failed to mmap buffer" << std::endl;
                return nullptr;
            }
        }
        
        return memory;
    }

    void unmapBuffer(libcamera::FrameBuffer* buffer, void* memory) {
        if (memory && buffer && !buffer->planes().empty()) {
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
                const auto &plane = buffer->planes()[0];
                int width = stream->configuration().size.width;
                int height = stream->configuration().size.height;
                int stride = stream->configuration().stride;
                
                // Calculate proper buffer size for YUYV format (2 bytes per pixel)
                size_t frameSize = height * stride;
                
                // Open file in binary mode
                std::ofstream file("captured_image.yuv", std::ios::binary);
                
                // Write frame data respecting stride
                const uint8_t* src = static_cast<const uint8_t*>(memory);
                for (int y = 0; y < height; y++) {
                    // Write one row at a time, using actual width (not stride)
                    file.write(reinterpret_cast<const char*>(src + y * stride), width * 2);
                }
                
                file.close();
                
                std::cout << "Image captured and saved as 'captured_image.yuv'" << std::endl;
                std::cout << "Resolution: " << width << "x" << height << std::endl;
                std::cout << "Stride: " << stride << " bytes" << std::endl;
                std::cout << "Frame size: " << frameSize << " bytes" << std::endl;
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
        cleanup();
    }
    
    bool initialize() {
        if (cameraManager_->start()) {
            std::cerr << "Failed to start camera manager" << std::endl;
            return false;
        }
        
        if (cameraManager_->cameras().empty()) {
            std::cerr << "No cameras available" << std::endl;
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
        
        // Configure stream
        libcamera::StreamConfiguration &streamConfig = config_->at(0);
        streamConfig.pixelFormat = libcamera::formats::YUYV;
        streamConfig.size.width = 1920;
        streamConfig.size.height = 1080;
        streamConfig.bufferCount = 1;
        streamConfig.stride = ((streamConfig.size.width * 2 + 31) & ~31);
        // Configure the camera
        libcamera::CameraConfiguration::Status status = config_->validate();
        if (status == libcamera::CameraConfiguration::Invalid) {
            std::cerr << "Camera configuration invalid" << std::endl;
            return false;
        }
        
        if (camera_->configure(config_.get()) < 0) {
            std::cerr << "Failed to configure camera" << std::endl;
            return false;
        }
        
        // Set up buffer allocator
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
        
        // Wait for sensor initialization
        std::this_thread::sleep_for(2s);
        
        // Start camera
        if (camera_->start()) {
            std::cerr << "Failed to start camera" << std::endl;
            return false;
        }
        cameraRunning_ = true;
        
        return true;
    }
    
    bool captureImage() {
        captureComplete_ = false;
        
        std::unique_ptr<libcamera::Request> request = camera_->createRequest();
        if (!request) {
            std::cerr << "Failed to create request" << std::endl;
            return false;
        }
        
        const libcamera::Stream *stream = config_->at(0).stream();
        libcamera::Stream *nonConstStream = const_cast<libcamera::Stream*>(stream);
        
        const std::vector<std::unique_ptr<libcamera::FrameBuffer>> &buffers = 
            allocator_->buffers(nonConstStream);
        
        if (request->addBuffer(stream, buffers[0].get()) < 0) {
            std::cerr << "Failed to add buffer to request" << std::endl;
            return false;
        }
        
        requests_.push_back(std::move(request));
        
        camera_->requestCompleted.connect(this, &StillCaptureHandler::requestComplete);
        
        if (camera_->queueRequest(requests_[0].get()) < 0) {
            std::cerr << "Failed to queue request" << std::endl;
            return false;
        }
        
        {
            std::unique_lock<std::mutex> lock(completionMutex_);
            if (!completionCondition_.wait_for(lock, 5s, 
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
            std::this_thread::sleep_for(100ms);
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
            
            for (auto &pair : mappedBuffers_) {
                unmapBuffer(pair.first, pair.second);
            }
            mappedBuffers_.clear();
            
            delete allocator_;
            allocator_ = nullptr;
            
            camera_->release();
        }
        
        if (cameraManager_) {
            cameraManager_->stop();
        }
    }
};

// Global capture handler
static std::unique_ptr<StillCaptureHandler> globalCaptureHandler;

void signalHandler(int signum) {
    std::cout << "\nReceived signal " << signum << std::endl;
    quit.store(true);
    if (globalCaptureHandler && globalCaptureHandler->isRunning()) {
        globalCaptureHandler->stopCamera();
    }
}

int main() {
    try {
        signal(SIGINT, signalHandler);
        signal(SIGTERM, signalHandler);
        
        globalCaptureHandler = std::make_unique<StillCaptureHandler>();
        
        if (!globalCaptureHandler->initialize()) {
            return -1;
        }
        
        std::cout << "Capturing image..." << std::endl;
        if (!globalCaptureHandler->captureImage()) {
            return -1;
        }
        
        std::this_thread::sleep_for(500ms);
        globalCaptureHandler.reset();
        
    } catch (const std::exception &e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        if (globalCaptureHandler) {
            globalCaptureHandler->stopCamera();
            globalCaptureHandler.reset();
        }
        return -1;
    }
    
    return 0;
}
