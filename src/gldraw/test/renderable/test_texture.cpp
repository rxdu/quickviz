/**
 * @file test_texture.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-06
 * @brief Test for Texture rendering functionality
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <memory>
#include <vector>
#include <filesystem>
#include <random>
#include <chrono>
#include <atomic>
#include <thread>
#include <cmath>

#include "gldraw/gl_view.hpp"
#include "gldraw/renderable/texture.hpp"
#include "core/buffer/buffer_registry.hpp"
#include "core/buffer/ring_buffer.hpp"

using namespace quickviz;
namespace fs = std::filesystem;

// Helper class to generate dynamic texture data
class DynamicTextureGenerator {
public:
    DynamicTextureGenerator(int width, int height)
        : width_(width), height_(height), gen_(rd_()), dist_(0, 255) {
        buffer_.resize(width * height * 4);  // RGBA format
    }

    std::vector<unsigned char> GenerateFrame(float time) {
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                float dx = x - width_ / 2.0f;
                float dy = y - height_ / 2.0f;
                float distance = std::sqrt(dx * dx + dy * dy);
                float angle = std::atan2(dy, dx);
                
                // Create moving circular patterns
                float pattern1 = std::sin(distance * 0.05f - time * 2.0f) * 0.5f + 0.5f;
                float pattern2 = std::cos(angle * 3.0f + time) * 0.5f + 0.5f;
                float pattern3 = std::sin(distance * 0.02f + angle * 2.0f - time) * 0.5f + 0.5f;
                
                int index = (y * width_ + x) * 4;
                buffer_[index] = static_cast<unsigned char>(pattern1 * 255);      // R
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

// Global variables for dynamic texture
const int TEX_WIDTH = 500;
const int TEX_HEIGHT = 500;
std::string buffer_name = "texture_buffer";
std::atomic<bool> running{true};
std::unique_ptr<std::thread> generate_thread;

// Function to generate texture data in a separate thread
void GenerateTextureData(const std::string& buffer_name, std::atomic<bool>& running) {
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
        
        // Cap the update rate (~60 FPS)
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
}

void SetupTextureScene(GlSceneManager* scene_manager) {
    // Set up buffer
    auto& buffer_registry = BufferRegistry::GetInstance();
    std::shared_ptr<BufferInterface<std::vector<unsigned char>>> texture_buffer =
        std::make_shared<RingBuffer<std::vector<unsigned char>, 8>>();
    buffer_registry.AddBuffer(buffer_name, texture_buffer);

    // Create and add texture
    auto texture = std::make_unique<Texture>();
    auto* texture_ptr = texture.get();
    scene_manager->AddOpenGLObject("texture", std::move(texture));

    // Configure texture
    texture_ptr->PreallocateBuffer(TEX_WIDTH, TEX_HEIGHT, Texture::PixelFormat::kRgba);
    texture_ptr->SetBufferUpdateStrategy(Texture::BufferUpdateStrategy::kAuto);
    texture_ptr->SetBufferUpdateThreshold(TEX_WIDTH * TEX_HEIGHT * 4 / 2);
    texture_ptr->SetOrigin(glm::vec3(-2.5f, -2.5f, 0.0f), 0.01f);  // 1cm per pixel

    // Set up pre-draw callback to update texture from buffer
    scene_manager->SetPreDrawCallback([texture_ptr, buffer_name]() {
        auto& buffer_registry = BufferRegistry::GetInstance();
        auto texture_buffer = buffer_registry.GetBuffer<std::vector<unsigned char>>(buffer_name);
        
        std::vector<unsigned char> data;
        if (texture_buffer->Read(data)) {
            texture_ptr->UpdateData(TEX_WIDTH, TEX_HEIGHT, Texture::PixelFormat::kRgba, std::move(data));
        }
    });

    // Start texture generation thread
    running = true;
    generate_thread = std::make_unique<std::thread>(GenerateTextureData, buffer_name, std::ref(running));
}

int main(int argc, char* argv[]) {
    try {
        // Configure the view for 2D mode
        GlView::Config config;
        config.window_title = "Texture Rendering Test - 2D Mode";
        config.scene_mode = GlSceneManager::Mode::k2D;
        config.coordinate_frame_size = 2.0f;
        
        // Create the view
        GlView view(config);
        
        // Set up description and help sections
        view.SetDescription("Testing dynamic texture rendering with animated patterns");
        
        view.AddHelpSection("Texture Features Demonstrated", {
            "- Dynamic texture generation with threading",
            "- Real-time texture updates at ~60 FPS",
            "- Complex mathematical patterns (circular waves)",
            "- RGBA format with full alpha support",
            "- Buffer management with ring buffer",
            "- Automatic buffer update strategy",
            "- Pre-draw callbacks for data updates"
        });
        
        view.AddHelpSection("Animation Patterns", {
            "- Pattern 1: Radial sine waves (Red channel)",
            "- Pattern 2: Angular cosine waves (Green channel)", 
            "- Pattern 3: Combined distance/angle sine (Blue channel)",
            "- All patterns animated with time parameter",
            "- Smooth color transitions and wave propagation",
            "- 500x500 pixel resolution"
        });
        
        view.AddHelpSection("Technical Details", {
            "- Buffer size: 500x500x4 bytes (RGBA)",
            "- Update rate: ~60 FPS (16ms per frame)",
            "- Threading: Background texture generation",
            "- Memory: Ring buffer with 8 frame capacity",
            "- Positioning: 1cm per pixel scale",
            "- Origin: (-2.5, -2.5, 0.0) meters"
        });
        
        view.AddHelpSection("Expected Visuals", {
            "- Continuously animated texture",
            "- Circular wave patterns radiating from center",
            "- Color variations: red, green, blue channels",
            "- Smooth transitions and movements",
            "- Grid and coordinate frame for reference",
            "- Real-time performance without stuttering"
        });
        
        // Set the scene setup callback
        view.SetSceneSetup(SetupTextureScene);
        
        std::cout << "\nStarting dynamic texture test..." << std::endl;
        std::cout << "You should see continuously animated patterns!" << std::endl;
        
        // Run the view
        view.Run();
        
        // Cleanup
        running = false;
        if (generate_thread && generate_thread->joinable()) {
            generate_thread->join();
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        
        // Cleanup on error
        running = false;
        if (generate_thread && generate_thread->joinable()) {
            generate_thread->join();
        }
        
        return 1;
    }
    
    return 0;
}