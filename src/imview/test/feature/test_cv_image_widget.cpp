/*
 * test_cv_image_widget.cpp
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
#include "imview/widget/cv_image_widget.hpp"

#include "scene_objects/gl_triangle_scene_object.hpp"

using namespace quickviz;

bool keep_running = true;
std::shared_ptr<CvImageWidget> image_widget;

void CaptureVideo() {
  cv::VideoCapture cap(0);  // Open the default camera
  if (!cap.isOpened()) {
    std::cerr << "Error: Could not open video capture device." << std::endl;
    return;
  }

  while (keep_running) {
    cv::Mat frame;
    cap >> frame;  // Capture a new frame
    if (frame.empty()) {
      break;  // End of video stream
    }

    if (image_widget != nullptr) image_widget->UpdateImage(frame);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(30));  // Simulate frame rate
  }
}

int main(int argc, char* argv[]) {
  // set up video capture thread --> producer
  std::thread capture_thread(CaptureVideo);

  // set up viewer --> consumer
  Viewer viewer;
  auto gl_triangle = std::make_shared<GLTriangleSceneObject>();
  viewer.AddSceneObject(gl_triangle);

  image_widget = std::make_shared<CvImageWidget>("camera");
  image_widget->OnResize(300, 200);
  image_widget->SetPosition(0, 0);
  viewer.AddSceneObject(image_widget);

  viewer.Show();

  // clean up
  keep_running = false;
  capture_thread.join();

  return 0;
}