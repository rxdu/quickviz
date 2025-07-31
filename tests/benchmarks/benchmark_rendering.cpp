/*
 * @file benchmark_rendering.cpp
 * @date 2024-06-25
 * @brief Performance benchmarks for rendering pipeline
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <benchmark/benchmark.h>
#include <memory>
#include <vector>
#include <random>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "renderer/gl_scene_manager.hpp"
#include "renderer/renderable/point_cloud.hpp"
#include "renderer/renderable/triangle.hpp"
#include "imview/viewer.hpp"
#include "core/buffer/ring_buffer.hpp"
#include "core/event/event.hpp"
#include "core/event/event_dispatcher.hpp"

using namespace quickviz;

class RenderingBenchmark : public ::benchmark::Fixture {
public:
    void SetUp(const ::benchmark::State& state) override {
        display_available_ = IsDisplayAvailable();
        
        if (display_available_) {
            try {
                viewer_ = std::make_unique<Viewer>("Benchmark", 1024, 768);
                scene_manager_ = std::make_shared<GlSceneManager>("BenchmarkScene");
                viewer_->AddSceneObject(scene_manager_);
            } catch (const std::runtime_error& e) {
                display_available_ = false;
                std::cerr << "Graphics initialization failed: " << e.what() << std::endl;
            }
        }
    }
    
protected:
    bool display_available_ = false;
    
private:
    bool IsDisplayAvailable() {
        const char* display = std::getenv("DISPLAY");
        if (!display || strlen(display) == 0) {
            return false;
        }
        return true;
    }

    void TearDown(const ::benchmark::State& state) override {
        scene_manager_.reset();
        viewer_.reset();
    }

protected:
    std::unique_ptr<Viewer> viewer_;
    std::shared_ptr<GlSceneManager> scene_manager_;
};

// Benchmark point cloud rendering with different sizes
BENCHMARK_DEFINE_F(RenderingBenchmark, PointCloudRendering)(benchmark::State& state) {
    if (!display_available_) {
        state.SkipWithError("No display available for graphics benchmark");
        return;
    }
    
    int point_count = state.range(0);
    
    // Generate random point cloud data
    std::vector<glm::vec4> points;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> pos_dist(-10.0f, 10.0f);
    
    points.reserve(point_count);
    
    for (int i = 0; i < point_count; ++i) {
        points.emplace_back(pos_dist(gen), pos_dist(gen), pos_dist(gen), 1.0f);
    }
    
    auto point_cloud = std::make_unique<PointCloud>();
    point_cloud->SetPoints(points, PointCloud::ColorMode::kStatic);
    scene_manager_->AddOpenGLObject("benchmark_points", std::move(point_cloud));
    
    for (auto _ : state) {
        // Simulate rendering frame
        viewer_->PollEvents();
        // Note: In a real benchmark, we would call the actual render function
        benchmark::DoNotOptimize(points.data());
    }
    
    state.SetItemsProcessed(state.iterations() * point_count);
}

BENCHMARK_REGISTER_F(RenderingBenchmark, PointCloudRendering)
    ->Args({1000})
    ->Args({10000})
    ->Args({100000})
    ->Args({1000000})
    ->Unit(benchmark::kMillisecond);

// Benchmark multiple triangle rendering
BENCHMARK_DEFINE_F(RenderingBenchmark, MultipleTriangleRendering)(benchmark::State& state) {
    if (!display_available_) {
        state.SkipWithError("No display available for graphics benchmark");
        return;
    }
    
    int triangle_count = state.range(0);
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> pos_dist(-5.0f, 5.0f);
    std::uniform_real_distribution<float> color_dist(0.0f, 1.0f);
    
    // Add multiple triangles
    for (int i = 0; i < triangle_count; ++i) {
        auto triangle = std::make_unique<Triangle>(
            1.0f, 
            glm::vec3(color_dist(gen), color_dist(gen), color_dist(gen))
        );
        
        scene_manager_->AddOpenGLObject("triangle_" + std::to_string(i), std::move(triangle));
    }
    
    for (auto _ : state) {
        viewer_->PollEvents();
        benchmark::DoNotOptimize(triangle_count);
    }
    
    state.SetItemsProcessed(state.iterations() * triangle_count);
}

BENCHMARK_REGISTER_F(RenderingBenchmark, MultipleTriangleRendering)
    ->Args({10})
    ->Args({100})
    ->Args({1000})
    ->Args({5000})
    ->Unit(benchmark::kMillisecond);

// Benchmark scene object management
static void BM_SceneObjectCreation(benchmark::State& state) {
    int object_count = state.range(0);
    
    for (auto _ : state) {
        std::vector<std::shared_ptr<GlSceneManager>> scenes;
        scenes.reserve(object_count);
        
        for (int i = 0; i < object_count; ++i) {
            auto scene = std::make_shared<GlSceneManager>("Scene" + std::to_string(i));
            scenes.push_back(scene);
        }
        
        benchmark::DoNotOptimize(scenes.data());
    }
    
    state.SetItemsProcessed(state.iterations() * object_count);
}

BENCHMARK(BM_SceneObjectCreation)
    ->Args({100})
    ->Args({1000})
    ->Args({10000})
    ->Unit(benchmark::kMicrosecond);

// Benchmark buffer operations
static void BM_RingBufferThroughput(benchmark::State& state) {
    int buffer_size = state.range(0);
    int operations = state.range(1);
    
    RingBuffer<int, 1024> buffer(true); // Enable overwrite
    
    for (auto _ : state) {
        for (int i = 0; i < operations; ++i) {
            buffer.Write(i);
            int value;
            buffer.Read(value);
        }
    }
    
    state.SetItemsProcessed(state.iterations() * operations * 2); // Write + Read
}

BENCHMARK(BM_RingBufferThroughput)
    ->Args({512, 1000})
    ->Args({1024, 1000})
    ->Args({2048, 1000})
    ->Args({1024, 10000})
    ->Unit(benchmark::kMicrosecond);

// Benchmark event system performance
static void BM_EventDispatcherThroughput(benchmark::State& state) {
    int event_count = state.range(0);
    
    std::atomic<int> received_count{0};
    
    EventDispatcher::GetInstance().RegisterHandler("benchmark_event",
        [&](std::shared_ptr<BaseEvent> event) {
            received_count++;
        }
    );
    
    for (auto _ : state) {
        received_count = 0;
        for (int i = 0; i < event_count; ++i) {
            auto event = std::make_shared<Event<int>>(EventSource::kApplicaton, "benchmark_event", i);
            EventDispatcher::GetInstance().Dispatch(event);
        }
        benchmark::DoNotOptimize(received_count.load());
    }
    
    state.SetItemsProcessed(state.iterations() * event_count);
}

BENCHMARK(BM_EventDispatcherThroughput)
    ->Args({1000})
    ->Args({10000})
    ->Args({100000})
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_MAIN();