/*
 * @file test_double_buffer.cpp
 * @date 10/8/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>
#include <thread>

#include "core/buffer/ring_buffer.hpp"

#include <opencv2/opencv.hpp>

using namespace quickviz;

using CvBuffer = RingBuffer<cv::Mat, 8>;

// Background thread function to capture video frames
void CaptureVideo(CvBuffer& buffer) {
  cv::VideoCapture cap(0);  // Open the default camera
  if (!cap.isOpened()) {
    std::cerr << "Error: Could not open video capture device." << std::endl;
    return;
  }

  while (true) {
    cv::Mat frame;
    cap >> frame;  // Capture a new frame
    if (frame.empty()) {
      break;  // End of video stream
    }

    buffer.Write(frame);  // Write frame to double buffer
    std::this_thread::sleep_for(
        std::chrono::milliseconds(30));  // Simulate frame rate
  }
}

// Main GUI thread function to display video frames
void DisplayVideo(CvBuffer& buffer) {
  while (true) {
    cv::Mat frame;
    buffer.Read(frame);  // Read frame from double buffer

    if (frame.empty()) {
      continue;
    }

    std::cout << "video size: " << frame.size() << std::endl;

    // Display the frame (assuming OpenCV is used for display)
    cv::imshow("Video Display", frame);

    if (cv::waitKey(1) == 27) {  // Exit on 'ESC' key
      break;
    }
  }
}

int main(int argc, char* argv[]) {
  CvBuffer buffer;

  // Start the video capture thread
  std::thread capture_thread(CaptureVideo, std::ref(buffer));

  // Start the video display in the main thread
  DisplayVideo(buffer);

  // Clean up
  capture_thread.join();

  return 0;
}