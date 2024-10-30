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
        
        // Map the first plane of the buffer
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
        if (memory) {
            const std::vector<libcamera::FrameBuffer::Plane>& planes = buffer->planes();
            if (!planes.empty()) {
                munmap(memory, planes[0].length);
            }
        }
    }
    
    // Callback function to handle completed requests
    void requestComplete(libcamera::Request *request) {
        if (request->status() == libcamera::Request::RequestComplete) {
            const libcamera::Request::BufferMap &buffers = request->buffers();
            for (auto &bufferPair : buffers) {
                libcamera::FrameBuffer *buffer = bufferPair.second;
                const libcamera::Stream *stream = bufferPair.first;
                
                // Get buffer data
                const auto &plane = buffer->planes()[0];
                void* memory = mappedBuffers_[buffer];
                
                if (memory) {
                    // Save the buffer to a file
                    std::ofstream file("captured_image.yuv", std::ios::binary);
                    file.write(static_cast<char*>(memory), plane.length);
                    file.close();
                    
                    std::cout << "Image captured and saved as 'captured_image.yuv'" << std::endl;
                    std::cout << "Resolution: " << stream->configuration().size.width;
                    std::cout << "x" << stream->configuration().size.height << std::endl;
                    std::cout << "Buffer size: " << plane.length << " bytes" << std::endl;
                }
            }
        } else {
            std::cerr << "Request failed!" << std::endl;
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
        
        std::vector<libcamera::StreamRole> roles = {libcamera::StreamRole::StillCapture};
        config_ = camera_->generateConfiguration(roles);
        if (!config_) {
            std::cerr << "Failed to generate configuration" << std::endl;
            return false;
        }
        
        // Configure stream for high resolution still capture
        libcamera::StreamConfiguration &streamConfig = config_->at(0);
        streamConfig.pixelFormat = libcamera::formats::YUYV;
        streamConfig.size.width = 640;  // Max resolution for IMX219
        streamConfig.size.height = 480;
        streamConfig.bufferCount = 1;
        
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

            // Map the allocated buffers
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
        // Create capture request
        std::unique_ptr<libcamera::Request> request = camera_->createRequest();
        if (!request) {
            std::cerr << "Failed to create request" << std::endl;
            return false;
        }
        
        // Get stream and cast to non-const
        const libcamera::Stream *constStream = config_->at(0).stream();
        libcamera::Stream *stream = const_cast<libcamera::Stream*>(constStream);
        
        const std::vector<std::unique_ptr<libcamera::FrameBuffer>> &buffers = allocator_->buffers(stream);
        if (request->addBuffer(constStream, buffers[0].get()) < 0) {
            std::cerr << "Failed to add buffer to request" << std::endl;
            return false;
        }
        
        requests_.push_back(std::move(request));
        
        // Setup completion handler
        camera_->requestCompleted.connect(this, &StillCaptureHandler::requestComplete);
        
        // Start camera
        if (camera_->start() < 0) {
            std::cerr << "Failed to start camera" << std::endl;
            return false;
        }
        cameraRunning_ = true;
        
        // Queue request
        if (camera_->queueRequest(requests_[0].get()) < 0) {
            std::cerr << "Failed to queue request" << std::endl;
            return false;
        }
        
        // Wait for capture to complete or quit signal
        std::unique_lock<std::mutex> lock(completionMutex_);
        completionCondition_.wait(lock, [this] { return captureComplete_ || quit.load(); });
        
        stopCamera();
        return captureComplete_;
    }
    
    void stopCamera() {
        if (cameraRunning_) {
            camera_->stop();
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
            
            // Unmap all buffers
            for (auto &pair : mappedBuffers_) {
                unmapBuffer(pair.first, pair.second);
            }
            mappedBuffers_.clear();
            
            camera_->release();
        }
        
        delete allocator_;
        allocator_ = nullptr;
        
        if (cameraManager_) {
            cameraManager_->stop();
        }
    }
};

// Global pointer for signal handler
static std::unique_ptr<StillCaptureHandler> globalCaptureHandler;

void signalHandler(int signum) {
    std::cout << "\nSignal (" << signum << ") received.\n";
    quit.store(true);
    if (globalCaptureHandler && globalCaptureHandler->isRunning()) {
        globalCaptureHandler->stopCamera();
    }
}

int main() {
    try {
        // Set up signal handling
        signal(SIGINT, signalHandler);
        signal(SIGTERM, signalHandler);
        
        // Create capture handler
        globalCaptureHandler = std::make_unique<StillCaptureHandler>();
        
        if (!globalCaptureHandler->initialize()) {
            return -1;
        }
        
        std::cout << "Capturing image..." << std::endl;
        if (!globalCaptureHandler->captureImage()) {
            return -1;
        }
        
        // Add delay before cleanup
        std::this_thread::sleep_for(1000ms);
        
        // Clean up global pointer
        globalCaptureHandler.reset();
        
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}
