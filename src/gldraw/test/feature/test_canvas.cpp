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

#include "imview/box.hpp"
#include "imview/viewer.hpp"

#include "gldraw/gl_scene_manager.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/triangle.hpp"
#include "gldraw/renderable/coordinate_frame.hpp"
#include "gldraw/renderable/canvas.hpp"

using namespace quickviz;
namespace fs = std::filesystem;

// Function to test all canvas drawing functions
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
  
  // Add a polygon
  std::vector<glm::vec2> star_vertices = {
    {0.0f, 5.0f},
    {1.0f, 2.0f},
    {4.0f, 2.0f},
    {2.0f, 0.0f},
    {3.0f, -3.0f},
    {0.0f, -1.0f},
    {-3.0f, -3.0f},
    {-2.0f, 0.0f},
    {-4.0f, 2.0f},
    {-1.0f, 2.0f},
  };
  
  // Scale down the star vertices
  for (auto& vertex : star_vertices) {
    vertex *= 0.3f;
  }
  
  // Move the star to a different position
  for (auto& vertex : star_vertices) {
    vertex += glm::vec2(4.0f, 3.0f);
  }
  
  canvas->AddPolygon(star_vertices, glm::vec4(0.8f, 0.8f, 0.0f, 0.9f), true, 2.0f);  // Gold filled
  
  // Add a simple test polygon that should be very visible
  std::vector<glm::vec2> test_polygon = {
    {-1.5f, -1.5f},  // Bottom left
    {-0.5f, -1.5f},  // Bottom right
    {-0.5f, -0.5f},  // Top right
    {-1.5f, -0.5f}   // Top left
  };
  canvas->AddPolygon(test_polygon, glm::vec4(1.0f, 0.0f, 1.0f, 1.0f), true, 3.0f);  // Bright magenta filled
}

int main(int argc, char* argv[]) {
  bool thread_test = false;
  bool performance_test = false;
  
  // Check for test flags
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--thread-test") {
      thread_test = true;
    } else if (std::string(argv[i]) == "--performance-test") {
      performance_test = true;
    }
  }

  Viewer viewer;

  // create a box to manage size & position of the OpenGL scene
  auto box = std::make_shared<Box>("main_box");
  box->SetFlexDirection(Styling::FlexDirection::kRow);
  box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
  box->SetAlignItems(Styling::AlignItems::kStretch);

  // create a OpenGL scene manager to manage the OpenGL objects
  auto gl_sm = std::make_shared<GlSceneManager>("OpenGL Scene (2D)",
                                                GlSceneManager::Mode::k2D);
  gl_sm->SetAutoLayout(true);
  gl_sm->SetNoTitleBar(true);
  gl_sm->SetFlexGrow(0.5f);
  gl_sm->SetFlexShrink(0.0f);

  // now add the rendering objects to the OpenGL scene manager
  auto triangle = std::make_unique<Triangle>(1.0f, glm::vec3(0.0f, 0.5f, 0.5f));
  gl_sm->AddOpenGLObject("triangle", std::move(triangle));

  auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  gl_sm->AddOpenGLObject("grid", std::move(grid));

  // Add a coordinate frame in 2D mode (should show X and Z axes)
  auto coord_frame = std::make_unique<CoordinateFrame>(0.5f, true);
  gl_sm->AddOpenGLObject("coordinate_frame", std::move(coord_frame));

  auto canvas = std::make_unique<Canvas>();
  gl_sm->AddOpenGLObject("canvas", std::move(canvas));

  // now let's do some drawing on the canvas
  {
    auto canvas = static_cast<Canvas*>(gl_sm->GetOpenGLObject("canvas"));

    // Add background image first so it's behind all other drawings
    std::string image_path = "../data/fish.png";
    
    // Check if file exists and get absolute path
    fs::path abs_path = fs::absolute(image_path);
    std::cout << "Checking image path: " << abs_path.string() << std::endl;
    if (fs::exists(abs_path)) {
      std::cout << "Image file exists!" << std::endl;
    } else {
      std::cout << "Image file does not exist!" << std::endl;
      
      // Try alternative paths
      std::string alt_path1 = "data/fish.png";
      fs::path abs_alt_path1 = fs::absolute(alt_path1);
      std::cout << "Trying alternative path: " << abs_alt_path1.string() << std::endl;
      if (fs::exists(abs_alt_path1)) {
        std::cout << "Alternative image file exists!" << std::endl;
        image_path = alt_path1;
      }
      
      std::string alt_path2 = "fish.png";
      fs::path abs_alt_path2 = fs::absolute(alt_path2);
      std::cout << "Trying alternative path: " << abs_alt_path2.string() << std::endl;
      if (fs::exists(abs_alt_path2)) {
        std::cout << "Alternative image file exists!" << std::endl;
        image_path = alt_path2;
      }
    }
    
    // Configure performance monitoring if performance test is enabled
    if (performance_test) {
      Canvas::PerformanceConfig perf_config;
      perf_config.detailed_timing_enabled = true;
      perf_config.memory_tracking_enabled = true;
      perf_config.aggressive_memory_cleanup = true;
      perf_config.stats_update_frequency = 10; // Update every 10 frames
      canvas->SetPerformanceConfig(perf_config);
      
      // Pre-allocate memory for better performance
      canvas->PreallocateMemory(1000); // Expect ~1000 objects
      
      std::cout << "\n=== Performance Test Mode Enabled ===" << std::endl;
      std::cout << "Detailed timing: ON" << std::endl;
      std::cout << "Memory tracking: ON" << std::endl;
      std::cout << "Aggressive cleanup: ON" << std::endl;
      std::cout << "Pre-allocated for 1000 objects" << std::endl;
      std::cout << "Initial memory usage: " << canvas->GetMemoryUsage() / 1024 << " KB" << std::endl;
    }
    
    // Add background image using a small origin offset and 1:100 resolution for debugging
    canvas->AddBackgroundImage(image_path, glm::vec3(1.0f, 1.0f, 0.785f), 0.005f);
    
    // Test all canvas drawing functions
    TestAllCanvasFunctions(canvas);
    
    // If performance test, add many more objects to stress test the system
    if (performance_test) {
      std::cout << "\n=== Adding Performance Test Objects ===" << std::endl;
      
      // Add many lines to test line batching
      for (int i = 0; i < 100; ++i) {
        float x1 = -5.0f + (i % 10);
        float y1 = -5.0f + (i / 10);
        float x2 = x1 + 0.5f;
        float y2 = y1 + 0.5f;
        canvas->AddLine(x1, y1, x2, y2, 
                       glm::vec4(0.5f, 0.5f + i * 0.005f, 0.8f, 0.7f), 
                       1.5f, LineType::kSolid);
      }
      
      // Add many rectangles to test shape batching
      for (int i = 0; i < 50; ++i) {
        float x = -3.0f + (i % 10) * 0.6f;
        float y = -3.0f + (i / 10) * 0.6f;
        canvas->AddRectangle(x, y, 0.4f, 0.4f,
                           glm::vec4(0.8f, 0.3f + i * 0.01f, 0.3f, 0.8f),
                           i % 2 == 0, 2.0f);
      }
      
      // Add many circles to test circle batching
      for (int i = 0; i < 50; ++i) {
        float x = 1.0f + (i % 10) * 0.8f;
        float y = 1.0f + (i / 10) * 0.8f;
        canvas->AddCircle(x, y, 0.3f,
                         glm::vec4(0.3f, 0.8f, 0.3f + i * 0.01f, 0.8f),
                         i % 2 == 0, 2.0f);
      }
      
      // Add some ellipses (individual rendering)
      for (int i = 0; i < 10; ++i) {
        float x = -5.0f + i * 1.0f;
        float y = 6.0f;
        canvas->AddEllipse(x, y, 0.4f, 0.2f, i * 0.3f, 0.0f, 6.28f,
                          glm::vec4(0.9f, 0.5f, 0.1f, 0.8f), true, 2.0f);
      }
      
      // Add some polygons (individual rendering)
      for (int i = 0; i < 10; ++i) {
        std::vector<glm::vec2> triangle = {
          {-5.0f + i * 1.0f, 8.0f},
          {-4.5f + i * 1.0f, 8.5f},
          {-5.5f + i * 1.0f, 8.5f}
        };
        canvas->AddPolygon(triangle, glm::vec4(0.7f, 0.2f, 0.9f, 0.8f), true, 2.0f);
      }
      
      std::cout << "Added 100 lines, 50 rectangles, 50 circles, 10 ellipses, 10 polygons" << std::endl;
      std::cout << "Expected: ~150 batched objects, ~20 individual objects" << std::endl;
      
      // FORCE RENDERING to collect real performance statistics
      std::cout << "\n=== Forcing Rendering for Performance Measurement ===" << std::endl;
      
      // Create dummy projection/view matrices for rendering
      glm::mat4 projection = glm::ortho(-10.0f, 10.0f, -10.0f, 10.0f, -1.0f, 1.0f);
      glm::mat4 view = glm::mat4(1.0f);
      glm::mat4 coord_transform = glm::mat4(1.0f);
      
      // Trigger multiple render calls to simulate real usage
      for (int i = 0; i < 10; ++i) {
        canvas->OnDraw(projection, view, coord_transform);
      }
      
      std::cout << "Completed 10 render calls to collect statistics" << std::endl;
    }
    
    // Print performance statistics if performance test is enabled
    if (performance_test) {
      // Force a render cycle by calling the update manually
      // (In a real app, this would happen in the render loop)
      std::cout << "\n=== Initial Performance Statistics ===" << std::endl;
      const auto& stats = canvas->GetRenderStats();
      std::cout << "Draw calls: " << stats.draw_calls << std::endl;
      std::cout << "State changes: " << stats.state_changes << std::endl;
      std::cout << "Batched objects: " << stats.batched_objects << std::endl;
      std::cout << "Individual objects: " << stats.individual_objects << std::endl;
      std::cout << "Batch efficiency: " << stats.batch_efficiency << "%" << std::endl;
      std::cout << "Memory usage: " << canvas->GetMemoryUsage() / 1024 << " KB" << std::endl;
      std::cout << "Vertex memory: " << stats.vertex_memory_used / 1024 << " KB" << std::endl;
      std::cout << "Index memory: " << stats.index_memory_used / 1024 << " KB" << std::endl;
      std::cout << "Total memory: " << stats.GetTotalMemoryMB() << " MB" << std::endl;
      
      // Performance evaluation and recommendations
      std::cout << "\n=== Performance Evaluation ===" << std::endl;
      
      // Evaluate batch efficiency
      if (stats.batch_efficiency >= 80.0f) {
        std::cout << "✅ EXCELLENT: Batch efficiency " << stats.batch_efficiency << "% (>= 80%)" << std::endl;
      } else if (stats.batch_efficiency >= 60.0f) {
        std::cout << "✅ GOOD: Batch efficiency " << stats.batch_efficiency << "% (>= 60%)" << std::endl;
      } else if (stats.batch_efficiency >= 40.0f) {
        std::cout << "⚠️  ACCEPTABLE: Batch efficiency " << stats.batch_efficiency << "% (>= 40%)" << std::endl;
        std::cout << "   Consider using more rectangles/circles vs ellipses/polygons" << std::endl;
      } else {
        std::cout << "❌ POOR: Batch efficiency " << stats.batch_efficiency << "% (< 40%)" << std::endl;
        std::cout << "   Too many individual shapes (ellipses/polygons). Use circles/rectangles when possible." << std::endl;
      }
      
      // Evaluate draw calls efficiency (estimate based on objects)
      uint32_t total_objects = stats.batched_objects + stats.individual_objects;
      float draw_call_efficiency = total_objects > 0 ? 
        (static_cast<float>(total_objects) / std::max(1u, stats.draw_calls)) : 0.0f;
      
      if (draw_call_efficiency >= 50.0f) {
        std::cout << "✅ EXCELLENT: Draw call efficiency " << draw_call_efficiency << " objects/call (>= 50)" << std::endl;
      } else if (draw_call_efficiency >= 20.0f) {
        std::cout << "✅ GOOD: Draw call efficiency " << draw_call_efficiency << " objects/call (>= 20)" << std::endl;
      } else if (draw_call_efficiency >= 5.0f) {
        std::cout << "⚠️  ACCEPTABLE: Draw call efficiency " << draw_call_efficiency << " objects/call (>= 5)" << std::endl;
      } else {
        std::cout << "❌ POOR: Draw call efficiency " << draw_call_efficiency << " objects/call (< 5)" << std::endl;
        std::cout << "   Enable batching or reduce individual rendering shapes." << std::endl;
      }
      
      // Evaluate memory usage
      size_t memory_kb = canvas->GetMemoryUsage() / 1024;
      if (memory_kb <= 100) {
        std::cout << "✅ EXCELLENT: Memory usage " << memory_kb << " KB (<= 100 KB)" << std::endl;
      } else if (memory_kb <= 500) {
        std::cout << "✅ GOOD: Memory usage " << memory_kb << " KB (<= 500 KB)" << std::endl;
      } else if (memory_kb <= 1000) {
        std::cout << "⚠️  ACCEPTABLE: Memory usage " << memory_kb << " KB (<= 1 MB)" << std::endl;
      } else {
        std::cout << "❌ HIGH: Memory usage " << memory_kb << " KB (> 1 MB)" << std::endl;
        std::cout << "   Consider calling OptimizeMemory() or reducing object count." << std::endl;
      }
      
      // Expected vs Actual comparison
      std::cout << "\n=== Expected vs Actual ===" << std::endl;
      std::cout << "Expected ~150 batched + ~20 individual = ~170 total objects" << std::endl;
      std::cout << "Actual: " << stats.batched_objects << " batched + " 
                << stats.individual_objects << " individual = " 
                << total_objects << " total objects" << std::endl;
      
      if (total_objects == 0) {
        std::cout << "❌ CRITICAL: No objects detected! Rendering may not be working." << std::endl;
        std::cout << "   Check if shapes are being added to canvas correctly." << std::endl;
      } else if (total_objects < 150) {
        std::cout << "⚠️  WARNING: Fewer objects than expected. Some may not be rendering." << std::endl;
      } else {
        std::cout << "✅ Object count looks reasonable." << std::endl;
      }
      
      // Overall performance grade
      std::cout << "\n=== Overall Performance Grade ===" << std::endl;
      int score = 0;
      if (stats.batch_efficiency >= 60.0f) score += 3;
      else if (stats.batch_efficiency >= 40.0f) score += 2;
      else if (stats.batch_efficiency >= 20.0f) score += 1;
      
      if (draw_call_efficiency >= 20.0f) score += 3;
      else if (draw_call_efficiency >= 5.0f) score += 2;
      else if (draw_call_efficiency >= 1.0f) score += 1;
      
      if (memory_kb <= 500) score += 2;
      else if (memory_kb <= 1000) score += 1;
      
      if (total_objects >= 150) score += 2;
      else if (total_objects >= 50) score += 1;
      
      if (score >= 8) {
        std::cout << "🌟 GRADE A: Excellent performance! (" << score << "/10)" << std::endl;
      } else if (score >= 6) {
        std::cout << "✅ GRADE B: Good performance (" << score << "/10)" << std::endl;
      } else if (score >= 4) {
        std::cout << "⚠️  GRADE C: Acceptable performance (" << score << "/10)" << std::endl;
      } else {
        std::cout << "❌ GRADE D: Performance needs improvement (" << score << "/10)" << std::endl;
      }
    }
  }

  // Create a second OpenGL scene manager for 3D mode
  auto gl_sm_3d = std::make_shared<GlSceneManager>("OpenGL Scene (3D)",
                                                   GlSceneManager::Mode::k3D);
  gl_sm_3d->SetAutoLayout(true);
  gl_sm_3d->SetNoTitleBar(true);
  gl_sm_3d->SetFlexGrow(0.5f);
  gl_sm_3d->SetFlexShrink(0.0f);

  // Add a grid for reference
  auto grid_3d =
      std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  gl_sm_3d->AddOpenGLObject("grid", std::move(grid_3d));

  // Add a coordinate frame in 3D mode
  auto coord_frame_3d = std::make_unique<CoordinateFrame>(3.0f, false);
  gl_sm_3d->AddOpenGLObject("coordinate_frame", std::move(coord_frame_3d));

  auto canvas_3d = std::make_unique<Canvas>();
  gl_sm_3d->AddOpenGLObject("canvas", std::move(canvas_3d));

  // now let's do some drawing on the canvas
  {
    auto canvas = static_cast<Canvas*>(gl_sm_3d->GetOpenGLObject("canvas"));
    
    // Add background image first
    canvas->AddBackgroundImage("../data/fish.png", glm::vec3(1.0f, 1.0f, 0.785f), 0.005f);
    
    // Test all canvas drawing functions in 3D view as well
    TestAllCanvasFunctions(canvas);
  }

  // If thread test is enabled, start a background thread that keeps adding shapes
  if (thread_test) {
    std::thread worker_thread([&gl_sm, &gl_sm_3d, performance_test]() {
      std::cout << "Starting thread safety test..." << std::endl;
      
      auto canvas_2d = static_cast<Canvas*>(gl_sm->GetOpenGLObject("canvas"));
      auto canvas_3d = static_cast<Canvas*>(gl_sm_3d->GetOpenGLObject("canvas"));
      
      // Enable performance monitoring for thread test
      if (performance_test) {
        Canvas::PerformanceConfig perf_config;
        perf_config.detailed_timing_enabled = true;
        perf_config.memory_tracking_enabled = true;
        perf_config.aggressive_memory_cleanup = false; // Don't cleanup during stress test
        canvas_2d->SetPerformanceConfig(perf_config);
        canvas_3d->SetPerformanceConfig(perf_config);
      }
      
      float x = 0.0f;
      float y = 0.0f;
      int iteration = 0;
      
      // Keep adding shapes in a loop
      while (true) {
        // Clear canvases occasionally to avoid cluttering
        if (iteration % 100 == 0) {
          canvas_2d->Clear();
          canvas_3d->Clear();
          std::cout << "Cleared canvases at iteration " << iteration << std::endl;
          
          // Print performance statistics periodically
          if (performance_test && iteration % 500 == 0 && iteration > 0) {
            const auto& stats_2d = canvas_2d->GetRenderStats();
            std::cout << "\n=== Thread Test Performance (2D) - Iteration " << iteration << " ===" << std::endl;
            std::cout << "FPS: " << stats_2d.GetFPS() << " (avg: " << stats_2d.GetAvgFPS() << ")" << std::endl;
            std::cout << "Draw calls: " << stats_2d.draw_calls << std::endl;
            std::cout << "Batch efficiency: " << stats_2d.batch_efficiency << "%" << std::endl;
            std::cout << "Memory usage: " << canvas_2d->GetMemoryUsage() / 1024 << " KB" << std::endl;
            
            // Trigger memory optimization occasionally
            if (iteration % 1000 == 0) {
              canvas_2d->OptimizeMemory();
              canvas_3d->OptimizeMemory();
              std::cout << "Memory optimization triggered" << std::endl;
            }
          }
        }
        
        // Add a variety of shapes with different parameters
        float angle = static_cast<float>(iteration) * 0.1f;
        
        // Calculate position along a spiral
        x = 5.0f * std::cos(angle) * (0.1f + 0.01f * iteration);
        y = 5.0f * std::sin(angle) * (0.1f + 0.01f * iteration);
        
        // Alternate between different shape types
        switch (iteration % 5) {
          case 0:
            canvas_2d->AddPoint(x, y, 
                              glm::vec4(std::sin(angle) * 0.5f + 0.5f, 
                                       std::cos(angle) * 0.5f + 0.5f, 
                                       0.5f, 1.0f), 
                              3.0f + std::sin(angle) * 2.0f);
            canvas_3d->AddPoint(x, y, 
                              glm::vec4(std::sin(angle) * 0.5f + 0.5f, 
                                       std::cos(angle) * 0.5f + 0.5f, 
                                       0.5f, 1.0f), 
                              3.0f + std::sin(angle) * 2.0f);
            break;
          case 1:
            canvas_2d->AddCircle(x, y, 0.3f + 0.1f * std::sin(angle), 
                               glm::vec4(0.8f, 0.2f, 0.2f, 0.7f), 
                               (iteration % 10) < 5, 2.0f);
            canvas_3d->AddCircle(x, y, 0.3f + 0.1f * std::sin(angle), 
                               glm::vec4(0.8f, 0.2f, 0.2f, 0.7f), 
                               (iteration % 10) < 5, 2.0f);
            break;
          case 2:
            canvas_2d->AddRectangle(x, y, 0.4f + 0.1f * std::sin(angle), 
                                  0.3f + 0.1f * std::cos(angle), 
                                  glm::vec4(0.2f, 0.8f, 0.2f, 0.7f), 
                                  (iteration % 10) >= 5, 2.0f);
            canvas_3d->AddRectangle(x, y, 0.4f + 0.1f * std::sin(angle), 
                                  0.3f + 0.1f * std::cos(angle), 
                                  glm::vec4(0.2f, 0.8f, 0.2f, 0.7f), 
                                  (iteration % 10) >= 5, 2.0f);
            break;
          case 3:
            canvas_2d->AddLine(x, y, x + std::cos(angle), y + std::sin(angle), 
                             glm::vec4(0.2f, 0.2f, 0.8f, 0.7f), 
                             1.0f + 0.5f * std::sin(angle), 
                             static_cast<LineType>((iteration / 20) % 3));
            canvas_3d->AddLine(x, y, x + std::cos(angle), y + std::sin(angle), 
                             glm::vec4(0.2f, 0.2f, 0.8f, 0.7f), 
                             1.0f + 0.5f * std::sin(angle), 
                             static_cast<LineType>((iteration / 20) % 3));
            break;
          case 4:
            canvas_2d->AddEllipse(x, y, 
                                0.5f + 0.1f * std::sin(angle), 
                                0.3f + 0.1f * std::cos(angle), 
                                angle * 0.5f, 0.0f, 6.28f, 
                                glm::vec4(0.8f, 0.8f, 0.2f, 0.7f), 
                                (iteration % 10) < 5, 2.0f);
            canvas_3d->AddEllipse(x, y, 
                                0.5f + 0.1f * std::sin(angle), 
                                0.3f + 0.1f * std::cos(angle), 
                                angle * 0.5f, 0.0f, 6.28f, 
                                glm::vec4(0.8f, 0.8f, 0.2f, 0.7f), 
                                (iteration % 10) < 5, 2.0f);
            break;
        }
        
        // Occasionally add a polygon
        if (iteration % 30 == 0) {
          std::vector<glm::vec2> polygon_vertices;
          int num_vertices = 3 + (iteration % 7); // 3 to 9 vertices
          
          for (int i = 0; i < num_vertices; ++i) {
            float vertex_angle = 2.0f * M_PI * i / num_vertices;
            float radius = 0.3f + 0.1f * std::sin(angle + i);
            polygon_vertices.push_back({
              x + radius * std::cos(vertex_angle), 
              y + radius * std::sin(vertex_angle)
            });
          }
          
          canvas_2d->AddPolygon(polygon_vertices, 
                              glm::vec4(0.2f, 0.8f, 0.8f, 0.7f), 
                              (iteration % 10) >= 5, 2.0f);
          canvas_3d->AddPolygon(polygon_vertices, 
                              glm::vec4(0.2f, 0.8f, 0.8f, 0.7f), 
                              (iteration % 10) >= 5, 2.0f);
        }
        
        // Sleep a little to avoid adding shapes too quickly
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        iteration++;
      }
    });
    
    // Detach the thread so it runs independently
    worker_thread.detach();
  }
  
  // finally pass the OpenGL scene managers to the box and add it to the viewer
  box->AddChild(gl_sm);
  box->AddChild(gl_sm_3d);
  viewer.AddSceneObject(box);

  viewer.Show();

  return 0;
}