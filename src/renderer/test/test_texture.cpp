/**
 * @file test_gl_scene_manager.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-06
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>
#include <thread>
#include <filesystem>
#include <random>
#include <chrono>
#include <atomic>

#include "imview/box.hpp"
#include "imview/viewer.hpp"

#include "renderer/gl_scene_manager.hpp"
#include "renderer/renderable/grid.hpp"
#include "renderer/renderable/coordinate_frame.hpp"
#include "renderer/renderable/texture.hpp"

#include "core/buffer/buffer_registry.hpp"
#include "core/buffer/ring_buffer.hpp"

using namespace quickviz;
namespace fs = std::filesystem;

// Helper class to generate dynamic texture data
class DynamicTextureGenerator {
 public:
  DynamicTextureGenerator(int width, int height)
      : width_(width), height_(height), gen_(rd_()), dist_(0, 255) {
    // Pre-allocate buffer
    buffer_.resize(width * height * 4);  // RGBA format
  }

  // Generate a new frame with moving patterns
  std::vector<unsigned char> GenerateFrame(float time) {
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        // Create dynamic patterns based on time and position
        float dx = x - width_ / 2.0f;
        float dy = y - height_ / 2.0f;
        float distance = std::sqrt(dx * dx + dy * dy);
        float angle = std::atan2(dy, dx);
        
        // Create moving circular patterns
        float pattern1 = std::sin(distance * 0.05f - time * 2.0f) * 0.5f + 0.5f;
        float pattern2 = std::cos(angle * 3.0f + time) * 0.5f + 0.5f;
        float pattern3 = std::sin(distance * 0.02f + angle * 2.0f - time) * 0.5f + 0.5f;
        
        // Combine patterns and convert to RGB
        int index = (y * width_ + x) * 4;
        buffer_[index] = static_cast<unsigned char>(pattern1 * 255);  // R
        buffer_[index + 1] = static_cast<unsigned char>(pattern2 * 255);  // G
        buffer_[index + 2] = static_cast<unsigned char>(pattern3 * 255);  // B
        buffer_[index + 3] = 255;  // A (fully opaque)
      }
    }
    return buffer_;
  }

 private:
  int width_, height_;
  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_int_distribution<> dist_;
  std::vector<unsigned char> buffer_;
};

// Function to generate texture data in a separate thread
void GenerateTextureData(const std::string& buffer_name, std::atomic<bool>& running) {
  const int TEX_WIDTH = 500;
  const int TEX_HEIGHT = 500;
  
  auto& buffer_registry = BufferRegistry::GetInstance();
  auto texture_buffer = buffer_registry.GetBuffer<std::vector<unsigned char>>(buffer_name);
  
  DynamicTextureGenerator generator(TEX_WIDTH, TEX_HEIGHT);
  auto start_time = std::chrono::high_resolution_clock::now();
  
  while (running) {
    auto now = std::chrono::high_resolution_clock::now();
    float time = std::chrono::duration<float>(now - start_time).count();
    
    // Generate new frame
    auto data = generator.GenerateFrame(time);
    
    // Write to buffer
    texture_buffer->Write(std::move(data));
    
    // Cap the update rate
    std::this_thread::sleep_for(std::chrono::milliseconds(16));  // ~60 FPS
  }
}

int main(int argc, char* argv[]) {
  // Set up buffer first
  const int TEX_WIDTH = 500;
  const int TEX_HEIGHT = 500;
  std::string buffer_name = "texture_buffer";
  
  auto& buffer_registry = BufferRegistry::GetInstance();
  std::shared_ptr<BufferInterface<std::vector<unsigned char>>> texture_buffer =
      std::make_shared<RingBuffer<std::vector<unsigned char>, 8>>();  // Triple buffering
  buffer_registry.AddBuffer(buffer_name, texture_buffer);

  Viewer viewer;

  // Create a box to manage size & position of the OpenGL scene
  auto box = std::make_shared<Box>("main_box");
  box->SetFlexDirection(Styling::FlexDirection::kRow);
  box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
  box->SetAlignItems(Styling::AlignItems::kStretch);

  // Create a OpenGL scene manager
  auto gl_sm = std::make_shared<GlSceneManager>("OpenGL Scene (2D)",
                                               GlSceneManager::Mode::k2D);
  gl_sm->SetAutoLayout(true);
  gl_sm->SetNoTitleBar(true);
  gl_sm->SetFlexGrow(1.0f);
  gl_sm->SetFlexShrink(1.0f);

  std::cout << "Setting up scene objects..." << std::endl;

  // Add a grid
  auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.5f, 0.5f, 0.5f));
  gl_sm->AddOpenGLObject("grid", std::move(grid));

  // Add coordinate frame
  auto coord_frame = std::make_unique<CoordinateFrame>(2.0f, true);
  gl_sm->AddOpenGLObject("coordinate_frame", std::move(coord_frame));

  // Create and add texture
  std::cout << "Creating dynamic texture..." << std::endl;
  auto texture = std::make_unique<Texture>();
  
  // Get pointer before moving
  auto* texture_ptr = texture.get();
  gl_sm->AddOpenGLObject("texture", std::move(texture));

  // Pre-allocate texture buffer and set update strategy
  texture_ptr->PreallocateBuffer(TEX_WIDTH, TEX_HEIGHT, Texture::PixelFormat::kRgba);
  texture_ptr->SetBufferUpdateStrategy(Texture::BufferUpdateStrategy::kAuto);
  texture_ptr->SetBufferUpdateThreshold(TEX_WIDTH * TEX_HEIGHT * 4 / 2);  // Half the texture size

  // Position the texture in the scene
  texture_ptr->SetOrigin(glm::vec3(-2.5f, -2.5f, 0.0f), 0.01f);  // 1cm per pixel

  // Set up pre-draw callback to update texture from buffer
  gl_sm->SetPreDrawCallback([texture_ptr, buffer_name]() {
    auto& buffer_registry = BufferRegistry::GetInstance();
    auto texture_buffer = buffer_registry.GetBuffer<std::vector<unsigned char>>(buffer_name);
    
    std::vector<unsigned char> data;
    if (texture_buffer->Read(data)) {
      texture_ptr->UpdateData(TEX_WIDTH, TEX_HEIGHT, Texture::PixelFormat::kRgba, std::move(data));
    }
  });

  // Add scene manager to box and to viewer
  box->AddChild(gl_sm);
  viewer.AddSceneObject(box);

  std::cout << "\nStarting texture generation thread..." << std::endl;
  std::atomic<bool> running{true};
  std::thread generate_thread(GenerateTextureData, buffer_name, std::ref(running));

  std::cout << "\nTest is running. You should see:" << std::endl;
  std::cout << "1. A gray grid (10m x 10m)" << std::endl;
  std::cout << "2. Coordinate axes (red = X, green = Y)" << std::endl;
  std::cout << "3. A dynamic texture with moving patterns" << std::endl;
  std::cout << "\nThe texture should update continuously with:" << std::endl;
  std::cout << "- Circular wave patterns" << std::endl;
  std::cout << "- Color transitions" << std::endl;
  std::cout << "- Smooth animations" << std::endl;

  viewer.Show();

  // Cleanup
  running = false;
  if (generate_thread.joinable()) {
    generate_thread.join();
  }

  return 0;
}