/*
 * test_image_widget.cpp
 *
 * Created on: Jul 27, 2021 09:07
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <iostream>
#include <thread>

#include <opencv2/opencv.hpp>

#include "imview/viewer.hpp"
#include "imview/box.hpp"
#include "imview/buffer/buffer_registry.hpp"
#include "imview/buffer/ring_buffer.hpp"
#include "imview/buffer/double_buffer.hpp"

#include "scene_objects/gl_triangle_scene_object.hpp"
#include "imview/widget/image_widget.hpp"

using namespace quickviz;

void CaptureVideo() {
  cv::VideoCapture cap(0);  // Open the default camera
  if (!cap.isOpened()) {
    std::cerr << "Error: Could not open video capture device." << std::endl;
    return;
  }

  auto& buffer_registry = BufferRegistry::GetInstance();
  auto cv_buffer = buffer_registry.GetBuffer<cv::Mat>("video_buffer");

  while (true) {
    cv::Mat frame;
    cap >> frame;  // Capture a new frame
    if (frame.empty()) {
      break;  // End of video stream
    }

    cv_buffer->Write(frame);  // Write frame to double buffer
    std::this_thread::sleep_for(
        std::chrono::milliseconds(30));  // Simulate frame rate
  }
}

int main(int argc, char* argv[]) {
  Viewer viewer;

  auto& buffer_registry = BufferRegistry::GetInstance();
  std::shared_ptr<BufferInterface<cv::Mat>> cv_buffer =
      std::make_shared<RingBuffer<cv::Mat, 8>>();
  buffer_registry.AddBuffer("video_buffer", cv_buffer);

  // Start the video capture thread
  std::thread capture_thread(CaptureVideo);

  auto gl_triangle = std::make_shared<GLTriangleSceneObject>();
  auto image_widget = std::make_shared<ImageWidget>();
  image_widget->OnResize(300, 200);
  image_widget->SetPosition(0, 0);
  viewer.AddSceneObject(gl_triangle);
  viewer.AddSceneObject(image_widget);

  viewer.Show();

  capture_thread.join();

  return 0;
}