/**
 * @file test_canvas.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-06
 * @brief Test for Canvas rendering functionality
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <memory>
#include <vector>
#include <filesystem>
#include <thread>
#include <cmath>

#include "gldraw/gl_view.hpp"
#include "gldraw/renderable/canvas.hpp"
#include "gldraw/renderable/triangle.hpp"

using namespace quickviz;
namespace fs = std::filesystem;

void TestAllCanvasFunctions(Canvas* canvas) {
    // Add some points with different colors and sizes
    canvas->AddPoint(0.0f, 0.0f, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), 5.0f);  // Red
    canvas->AddPoint(1.0f, 1.0f, glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), 8.0f);  // Green
    canvas->AddPoint(-1.5f, -1.5f, glm::vec4(0.0f, 0.0f, 1.0f, 1.0f), 10.0f);  // Blue

    // Add lines with different styles
    canvas->AddLine(2.0f, 2.0f, 3.0f, 3.0f, glm::vec4(1.0f, 1.0f, 0.0f, 1.0f), 2.0f, LineType::kSolid);  // Yellow solid
    canvas->AddLine(-2.0f, 2.0f, -3.0f, 3.0f, glm::vec4(1.0f, 0.0f, 1.0f, 1.0f), 3.0f, LineType::kDashed);  // Magenta dashed
    canvas->AddLine(3.0f, -2.0f, 4.0f, -3.0f, glm::vec4(0.0f, 1.0f, 1.0f, 1.0f), 4.0f, LineType::kDotted);  // Cyan dotted

    // Add rectangles - filled and outlined
    canvas->AddRectangle(-4.0f, -4.0f, 1.0f, 1.0f, glm::vec4(1.0f, 0.5f, 0.0f, 0.7f), true, 2.0f);  // Orange filled
    canvas->AddRectangle(3.0f, -4.0f, 1.0f, 1.0f, glm::vec4(0.5f, 0.0f, 0.5f, 0.7f), false, 2.0f);  // Purple outlined

    // Add circles - filled and outlined
    canvas->AddCircle(-2.0f, -2.0f, 0.7f, glm::vec4(0.0f, 0.5f, 0.0f, 0.8f), true, 2.0f);  // Dark green filled
    canvas->AddCircle(2.0f, 0.0f, 0.5f, glm::vec4(0.7f, 0.7f, 0.7f, 0.8f), false, 2.0f);  // Gray outlined

    // Add ellipses - filled and outlined
    canvas->AddEllipse(0.0f, 3.0f, 1.0f, 0.5f, 0.0f, 0.0f, 6.28f, 
                      glm::vec4(0.5f, 0.5f, 0.0f, 0.8f), true, 2.0f);  // Olive filled
    canvas->AddEllipse(-3.0f, 0.0f, 0.7f, 0.4f, 0.7f, 0.0f, 6.28f, 
                      glm::vec4(0.5f, 0.0f, 0.0f, 0.8f), false, 2.0f);  // Dark red outlined, rotated

    // Add a star polygon
    std::vector<glm::vec2> star_vertices = {
        {0.0f, 5.0f}, {1.0f, 2.0f}, {4.0f, 2.0f}, {2.0f, 0.0f}, {3.0f, -3.0f},
        {0.0f, -1.0f}, {-3.0f, -3.0f}, {-2.0f, 0.0f}, {-4.0f, 2.0f}, {-1.0f, 2.0f}
    };
    
    // Scale down and reposition the star
    for (auto& vertex : star_vertices) {
        vertex = vertex * 0.3f + glm::vec2(4.0f, 3.0f);
    }
    canvas->AddPolygon(star_vertices, glm::vec4(0.8f, 0.8f, 0.0f, 0.9f), true, 2.0f);  // Gold filled

    // Add a simple visible polygon
    std::vector<glm::vec2> test_polygon = {
        {-1.5f, -1.5f}, {-0.5f, -1.5f}, {-0.5f, -0.5f}, {-1.5f, -0.5f}
    };
    canvas->AddPolygon(test_polygon, glm::vec4(1.0f, 0.0f, 1.0f, 1.0f), true, 3.0f);  // Bright magenta filled
}

void SetupCanvasScene(GlSceneManager* scene_manager) {
    // Add a triangle for reference
    auto triangle = std::make_unique<Triangle>(1.0f, glm::vec3(0.0f, 0.5f, 0.5f));
    scene_manager->AddOpenGLObject("triangle", std::move(triangle));

    // Create and configure canvas
    auto canvas = std::make_unique<Canvas>();
    scene_manager->AddOpenGLObject("canvas", std::move(canvas));

    // Get the canvas and add shapes
    auto canvas_ptr = static_cast<Canvas*>(scene_manager->GetOpenGLObject("canvas"));
    
    // Try to add background image
    std::string image_path = "../data/fish.png";
    fs::path abs_path = fs::absolute(image_path);
    
    if (!fs::exists(abs_path)) {
        // Try alternative paths
        std::vector<std::string> alt_paths = {"data/fish.png", "fish.png"};
        for (const auto& alt_path : alt_paths) {
            if (fs::exists(fs::absolute(alt_path))) {
                image_path = alt_path;
                break;
            }
        }
    }
    
    // Add background image with small scale for debugging
    canvas_ptr->AddBackgroundImage(image_path, glm::vec3(1.0f, 1.0f, 0.785f), 0.005f);
    
    // Test all canvas drawing functions
    TestAllCanvasFunctions(canvas_ptr);
}

int main(int argc, char* argv[]) {
    try {
        bool performance_test = false;
        
        // Check for performance test flag
        for (int i = 1; i < argc; ++i) {
            if (std::string(argv[i]) == "--performance-test") {
                performance_test = true;
            }
        }
        
        // Configure the view for 2D mode (canvas works best in 2D)
        GlView::Config config;
        config.window_title = "Canvas Rendering Test - 2D Mode";
        config.scene_mode = GlSceneManager::Mode::k2D;
        config.coordinate_frame_size = 0.5f;
        
        // Create the view
        GlView view(config);
        
        // Set up description and help sections
        view.SetDescription("Testing canvas 2D drawing functionality with various shapes and styles");
        
        view.AddHelpSection("Canvas Features Demonstrated", {
            "- Points with different colors and sizes",
            "- Lines: solid, dashed, and dotted styles",
            "- Rectangles: filled and outlined",
            "- Circles: filled and outlined",
            "- Ellipses: filled, outlined, and rotated",
            "- Polygons: star and simple shapes",
            "- Background image support",
            "- 2D rendering optimized for UI elements"
        });
        
        view.AddHelpSection("Shape Types", {
            "- Red/Green/Blue points (varying sizes)",
            "- Yellow solid line",
            "- Magenta dashed line", 
            "- Cyan dotted line",
            "- Orange filled rectangle",
            "- Purple outlined rectangle",
            "- Green filled circle",
            "- Gray outlined circle",
            "- Olive filled ellipse",
            "- Dark red outlined rotated ellipse",
            "- Gold star polygon",
            "- Magenta test square polygon"
        });
        
        view.AddHelpSection("Usage Notes", {
            "- Canvas works best in 2D mode for UI elements",
            "- Shapes support transparency (alpha channel)",
            "- Line styles: solid, dashed, dotted",
            "- Polygons can be complex shapes with multiple vertices",
            "- Background images are scaled and positioned automatically",
            "- Use --performance-test flag for performance analysis"
        });
        
        if (performance_test) {
            view.AddHelpSection("Performance Test Mode", {
                "- Detailed timing enabled",
                "- Memory tracking active", 
                "- Additional stress test shapes added",
                "- Batch efficiency measurements",
                "- Memory usage optimization",
                "- Draw call analysis"
            });
        }
        
        // Set the scene setup callback
        view.SetSceneSetup([performance_test](GlSceneManager* scene_manager) {
            SetupCanvasScene(scene_manager);
            
            // Add performance test objects if enabled
            if (performance_test) {
                auto canvas_ptr = static_cast<Canvas*>(scene_manager->GetOpenGLObject("canvas"));
                
                // Configure performance monitoring
                Canvas::PerformanceConfig perf_config;
                perf_config.detailed_timing_enabled = true;
                perf_config.memory_tracking_enabled = true;
                perf_config.aggressive_memory_cleanup = true;
                perf_config.stats_update_frequency = 10;
                canvas_ptr->SetPerformanceConfig(perf_config);
                canvas_ptr->PreallocateMemory(1000);
                
                std::cout << "\n=== Performance Test Mode Enabled ===" << std::endl;
                
                // Add many shapes for stress testing
                for (int i = 0; i < 100; ++i) {
                    float x = -5.0f + (i % 10);
                    float y = -5.0f + (i / 10);
                    canvas_ptr->AddLine(x, y, x + 0.5f, y + 0.5f, 
                                       glm::vec4(0.5f, 0.5f + i * 0.005f, 0.8f, 0.7f), 
                                       1.5f, LineType::kSolid);
                }
                
                for (int i = 0; i < 50; ++i) {
                    float x = -3.0f + (i % 10) * 0.6f;
                    float y = -3.0f + (i / 10) * 0.6f;
                    canvas_ptr->AddRectangle(x, y, 0.4f, 0.4f,
                                           glm::vec4(0.8f, 0.3f + i * 0.01f, 0.3f, 0.8f),
                                           i % 2 == 0, 2.0f);
                }
                
                for (int i = 0; i < 50; ++i) {
                    float x = 1.0f + (i % 10) * 0.8f;
                    float y = 1.0f + (i / 10) * 0.8f;
                    canvas_ptr->AddCircle(x, y, 0.3f,
                                         glm::vec4(0.3f, 0.8f, 0.3f + i * 0.01f, 0.8f),
                                         i % 2 == 0, 2.0f);
                }
                
                std::cout << "Added 100 lines, 50 rectangles, 50 circles for stress testing" << std::endl;
                
                // Force rendering to collect statistics
                glm::mat4 projection = glm::ortho(-10.0f, 10.0f, -10.0f, 10.0f, -1.0f, 1.0f);
                glm::mat4 view_matrix = glm::mat4(1.0f);
                glm::mat4 coord_transform = glm::mat4(1.0f);
                
                for (int i = 0; i < 10; ++i) {
                    canvas_ptr->OnDraw(projection, view_matrix, coord_transform);
                }
                
                // Print performance statistics
                const auto& stats = canvas_ptr->GetRenderStats();
                std::cout << "\n=== Performance Statistics ===" << std::endl;
                std::cout << "Draw calls: " << stats.draw_calls << std::endl;
                std::cout << "Batched objects: " << stats.batched_objects << std::endl;
                std::cout << "Individual objects: " << stats.individual_objects << std::endl;
                std::cout << "Batch efficiency: " << stats.batch_efficiency << "%" << std::endl;
                std::cout << "Memory usage: " << canvas_ptr->GetMemoryUsage() / 1024 << " KB" << std::endl;
            }
        });
        
        // Run the view
        view.Run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}