/**
 * @file profile_canvas_performance.cpp
 * @brief Performance profiling for Canvas implementation
 * @date 2025-08-26
 * 
 * This program profiles Canvas performance to validate whether decomposition
 * is necessary or if the current architecture already delivers target performance.
 */

#include <chrono>
#include <iostream>
#include <vector>
#include <random>
#include <memory>
#include <iomanip>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "scene/renderable/canvas.hpp"

using namespace quickviz;
using namespace std::chrono;

class CanvasProfiler {
public:
    struct PerformanceResults {
        // Workload metrics
        uint32_t total_shapes = 0;
        uint32_t points = 0;
        uint32_t lines = 0; 
        uint32_t rectangles = 0;
        uint32_t circles = 0;
        
        // Performance metrics
        double setup_time_ms = 0.0;
        double render_time_ms = 0.0;
        double avg_frame_time_ms = 0.0;
        double fps = 0.0;
        
        // Canvas metrics (if available)
        uint32_t draw_calls = 0;
        uint32_t batched_objects = 0;
        uint32_t individual_objects = 0;
        float batch_efficiency = 0.0f;
        size_t memory_usage_kb = 0;
        
        // Comparison metrics
        double shapes_per_ms = 0.0;
        double memory_per_shape_bytes = 0.0;
    };
    
    CanvasProfiler() {
        InitializeOpenGL();
        canvas_ = std::make_unique<Canvas>();
        
        // Setup matrices for rendering
        projection_ = glm::ortho(-50.0f, 50.0f, -50.0f, 50.0f, -1.0f, 1.0f);
        view_ = glm::mat4(1.0f);
        coord_transform_ = glm::mat4(1.0f);
    }
    
    ~CanvasProfiler() {
        canvas_.reset();
        if (window_) {
            glfwDestroyWindow(window_);
        }
        glfwTerminate();
    }
    
    PerformanceResults ProfileWorkload(const std::string& test_name, 
                                     uint32_t points, uint32_t lines, 
                                     uint32_t rectangles, uint32_t circles) {
        std::cout << "\n=== " << test_name << " ===" << std::endl;
        std::cout << "Workload: " << points << " points, " << lines << " lines, " 
                  << rectangles << " rects, " << circles << " circles" << std::endl;
        
        PerformanceResults results;
        results.points = points;
        results.lines = lines;
        results.rectangles = rectangles;
        results.circles = circles;
        results.total_shapes = points + lines + rectangles + circles;
        
        // Setup Canvas with performance monitoring
        Canvas::PerformanceConfig perf_config;
        perf_config.detailed_timing_enabled = true;
        perf_config.memory_tracking_enabled = true;
        perf_config.auto_batching_enabled = true;
        canvas_->SetPerformanceConfig(perf_config);
        canvas_->PreallocateMemory(results.total_shapes);
        canvas_->Clear();
        
        // Measure setup time
        auto setup_start = high_resolution_clock::now();
        GenerateWorkload(points, lines, rectangles, circles);
        auto setup_end = high_resolution_clock::now();
        results.setup_time_ms = duration<double, std::milli>(setup_end - setup_start).count();
        
        // Warm-up renders
        for (int i = 0; i < 5; ++i) {
            canvas_->OnDraw(projection_, view_, coord_transform_);
        }
        
        // Measure render performance
        const int render_iterations = 100;
        auto render_start = high_resolution_clock::now();
        
        for (int i = 0; i < render_iterations; ++i) {
            canvas_->OnDraw(projection_, view_, coord_transform_);
        }
        
        auto render_end = high_resolution_clock::now();
        results.render_time_ms = duration<double, std::milli>(render_end - render_start).count();
        results.avg_frame_time_ms = results.render_time_ms / render_iterations;
        results.fps = 1000.0 / results.avg_frame_time_ms;
        
        // Get Canvas statistics
        const auto& stats = canvas_->GetRenderStats();
        results.draw_calls = stats.draw_calls;
        results.batched_objects = stats.batched_objects;
        results.individual_objects = stats.individual_objects;
        results.batch_efficiency = stats.batch_efficiency;
        results.memory_usage_kb = canvas_->GetMemoryUsage() / 1024;
        
        // Calculate derived metrics
        results.shapes_per_ms = results.total_shapes / results.avg_frame_time_ms;
        results.memory_per_shape_bytes = (results.memory_usage_kb * 1024.0) / results.total_shapes;
        
        PrintResults(results);
        return results;
    }
    
    void RunComprehensiveProfile() {
        std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║                    CANVAS PERFORMANCE PROFILE                   ║" << std::endl;
        std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
        
        std::vector<PerformanceResults> all_results;
        
        // Test 1: Small workload (typical UI)
        all_results.push_back(ProfileWorkload("Small Workload (UI)", 50, 100, 50, 25));
        
        // Test 2: Medium workload (dashboard)
        all_results.push_back(ProfileWorkload("Medium Workload (Dashboard)", 200, 500, 200, 100));
        
        // Test 3: Large workload (data visualization)
        all_results.push_back(ProfileWorkload("Large Workload (Data Viz)", 1000, 2000, 500, 250));
        
        // Test 4: Stress test (maximum reasonable)
        all_results.push_back(ProfileWorkload("Stress Test (Maximum)", 2000, 5000, 1000, 500));
        
        // Test 5: Points-heavy (robotics point data)
        all_results.push_back(ProfileWorkload("Points Heavy (Robotics)", 5000, 100, 50, 25));
        
        // Test 6: Lines-heavy (path visualization)
        all_results.push_back(ProfileWorkload("Lines Heavy (Paths)", 100, 10000, 50, 25));
        
        PrintSummaryAnalysis(all_results);
    }
    
private:
    GLFWwindow* window_ = nullptr;
    std::unique_ptr<Canvas> canvas_;
    glm::mat4 projection_, view_, coord_transform_;
    std::mt19937 rng_{std::random_device{}()};
    
    void InitializeOpenGL() {
        if (!glfwInit()) {
            throw std::runtime_error("Failed to initialize GLFW");
        }
        
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE); // Hidden window for profiling
        
        window_ = glfwCreateWindow(800, 600, "Canvas Profiler", nullptr, nullptr);
        if (!window_) {
            glfwTerminate();
            throw std::runtime_error("Failed to create OpenGL context");
        }
        
        glfwMakeContextCurrent(window_);
        
        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
            throw std::runtime_error("Failed to initialize OpenGL loader");
        }
        
        std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;
        std::cout << "OpenGL Vendor: " << glGetString(GL_VENDOR) << std::endl;
        std::cout << "OpenGL Renderer: " << glGetString(GL_RENDERER) << std::endl;
    }
    
    void GenerateWorkload(uint32_t points, uint32_t lines, uint32_t rectangles, uint32_t circles) {
        std::uniform_real_distribution<float> pos_dist(-40.0f, 40.0f);
        std::uniform_real_distribution<float> size_dist(0.1f, 2.0f);
        std::uniform_real_distribution<float> color_dist(0.2f, 1.0f);
        
        // Generate points
        for (uint32_t i = 0; i < points; ++i) {
            canvas_->AddPoint(pos_dist(rng_), pos_dist(rng_), 
                            glm::vec4(color_dist(rng_), color_dist(rng_), color_dist(rng_), 0.8f),
                            size_dist(rng_) * 5.0f);
        }
        
        // Generate lines
        for (uint32_t i = 0; i < lines; ++i) {
            LineType line_type = static_cast<LineType>(i % 3); // Cycle through line types
            canvas_->AddLine(pos_dist(rng_), pos_dist(rng_), 
                           pos_dist(rng_), pos_dist(rng_),
                           glm::vec4(color_dist(rng_), color_dist(rng_), color_dist(rng_), 0.8f),
                           size_dist(rng_), line_type);
        }
        
        // Generate rectangles
        for (uint32_t i = 0; i < rectangles; ++i) {
            bool filled = (i % 2 == 0);
            canvas_->AddRectangle(pos_dist(rng_), pos_dist(rng_), 
                                size_dist(rng_) * 2.0f, size_dist(rng_) * 2.0f,
                                glm::vec4(color_dist(rng_), color_dist(rng_), color_dist(rng_), 0.7f),
                                filled, size_dist(rng_));
        }
        
        // Generate circles
        for (uint32_t i = 0; i < circles; ++i) {
            bool filled = (i % 2 == 0);
            canvas_->AddCircle(pos_dist(rng_), pos_dist(rng_), size_dist(rng_),
                             glm::vec4(color_dist(rng_), color_dist(rng_), color_dist(rng_), 0.7f),
                             filled, size_dist(rng_));
        }
    }
    
    void PrintResults(const PerformanceResults& results) {
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Setup Time: " << results.setup_time_ms << "ms" << std::endl;
        std::cout << "Avg Frame Time: " << results.avg_frame_time_ms << "ms" << std::endl;
        std::cout << "FPS: " << results.fps << std::endl;
        std::cout << "Draw Calls: " << results.draw_calls << std::endl;
        std::cout << "Batch Efficiency: " << results.batch_efficiency << "%" << std::endl;
        std::cout << "Memory Usage: " << results.memory_usage_kb << "KB" << std::endl;
        std::cout << "Performance: " << results.shapes_per_ms << " shapes/ms" << std::endl;
        std::cout << "Memory/Shape: " << results.memory_per_shape_bytes << " bytes" << std::endl;
    }
    
    void PrintSummaryAnalysis(const std::vector<PerformanceResults>& all_results) {
        std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║                        SUMMARY ANALYSIS                         ║" << std::endl;
        std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
        
        std::cout << std::fixed << std::setprecision(2);
        
        // Performance targets from refactor plan
        const double target_fps = 60.0;
        const double target_frame_time = 16.67; // ms
        
        std::cout << "\n📊 PERFORMANCE TARGETS vs ACTUAL:" << std::endl;
        std::cout << "Target: 60 FPS (16.67ms frame time)" << std::endl;
        
        bool all_meet_target = true;
        for (size_t i = 0; i < all_results.size(); ++i) {
            const auto& result = all_results[i];
            bool meets_target = result.fps >= target_fps;
            all_meet_target = all_meet_target && meets_target;
            
            std::cout << "Test " << (i+1) << ": " << result.fps << " FPS (" 
                      << result.avg_frame_time_ms << "ms) - " 
                      << (meets_target ? "✅ MEETS TARGET" : "❌ BELOW TARGET") << std::endl;
        }
        
        std::cout << "\n🎯 OVERALL PERFORMANCE ASSESSMENT:" << std::endl;
        if (all_meet_target) {
            std::cout << "✅ Canvas ALREADY MEETS all performance targets!" << std::endl;
            std::cout << "✅ Decomposition is NOT needed for performance reasons." << std::endl;
        } else {
            std::cout << "⚠️ Some tests below target - investigate bottlenecks." << std::endl;
        }
        
        // Batching efficiency analysis
        std::cout << "\n⚡ BATCHING EFFICIENCY ANALYSIS:" << std::endl;
        double avg_batch_efficiency = 0.0;
        double avg_shapes_per_draw_call = 0.0;
        for (const auto& result : all_results) {
            avg_batch_efficiency += result.batch_efficiency;
            if (result.draw_calls > 0) {
                avg_shapes_per_draw_call += static_cast<double>(result.total_shapes) / result.draw_calls;
            }
        }
        avg_batch_efficiency /= all_results.size();
        avg_shapes_per_draw_call /= all_results.size();
        
        std::cout << "Average Batch Efficiency: " << avg_batch_efficiency << "%" << std::endl;
        std::cout << "Average Shapes per Draw Call: " << avg_shapes_per_draw_call << std::endl;
        
        if (avg_batch_efficiency > 80.0) {
            std::cout << "✅ Excellent batching efficiency - optimization working well!" << std::endl;
        } else if (avg_batch_efficiency > 50.0) {
            std::cout << "⚠️ Good batching efficiency - room for minor improvements." << std::endl;
        } else {
            std::cout << "❌ Low batching efficiency - investigate batching logic." << std::endl;
        }
        
        // Memory efficiency analysis
        std::cout << "\n💾 MEMORY EFFICIENCY ANALYSIS:" << std::endl;
        double avg_memory_per_shape = 0.0;
        for (const auto& result : all_results) {
            avg_memory_per_shape += result.memory_per_shape_bytes;
        }
        avg_memory_per_shape /= all_results.size();
        
        std::cout << "Average Memory per Shape: " << avg_memory_per_shape << " bytes" << std::endl;
        
        if (avg_memory_per_shape < 100.0) {
            std::cout << "✅ Excellent memory efficiency!" << std::endl;
        } else if (avg_memory_per_shape < 200.0) {
            std::cout << "⚠️ Good memory efficiency - acceptable overhead." << std::endl;
        } else {
            std::cout << "❌ High memory overhead - investigate memory management." << std::endl;
        }
        
        // Final recommendation
        std::cout << "\n🎯 REFACTOR PLAN RECOMMENDATION:" << std::endl;
        if (all_meet_target && avg_batch_efficiency > 70.0 && avg_memory_per_shape < 150.0) {
            std::cout << "🚫 Canvas decomposition is NOT RECOMMENDED" << std::endl;
            std::cout << "   • Performance targets already met" << std::endl;
            std::cout << "   • Batching system working effectively" << std::endl;
            std::cout << "   • Memory usage is efficient" << std::endl;
            std::cout << "   • Focus on other optimization areas instead" << std::endl;
        } else {
            std::cout << "⚠️ Consider targeted optimizations:" << std::endl;
            if (!all_meet_target) std::cout << "   • Performance bottlenecks exist" << std::endl;
            if (avg_batch_efficiency <= 70.0) std::cout << "   • Improve batching efficiency" << std::endl;
            if (avg_memory_per_shape >= 150.0) std::cout << "   • Optimize memory usage" << std::endl;
        }
    }
};

int main(int argc, char* argv[]) {
    try {
        CanvasProfiler profiler;
        profiler.RunComprehensiveProfile();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Profiling failed: " << e.what() << std::endl;
        return 1;
    }
}