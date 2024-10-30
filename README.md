1.  To execute code for capturing video, use the command-
      g++ -std=c++17 -o video_cap video_cap.cpp -I/usr/include/libcamera -L/usr/lib -lcamera -ljpeg -lcamera-base -lpthread `pkg-config --cflags --libs opencv4`
      code name- video_cap.cpp

2.  To execute code for capturing still image, use the command-
       g++ -std=c++17 -o jpegCap6 jpegCap6.cpp -I/usr/include/libcamera -L/usr/lib -lcamera -ljpeg -lcamera-base -lpthread
    code name- jpegCap6.cpp
