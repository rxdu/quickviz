/*
 * test_point_cloud_buffer_strategies.cpp
 *
 * Created on: Jul 27, 2021 09:07
 * Description: Test different buffer update strategies for point clouds
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>
#include <thread>
#include <random>
#include <chrono>
#include <vector>
#include <numeric>
#include <iomanip>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>

#include "imview/box.hpp"
#include "imview/viewer.hpp"
#include "imview/component/opengl/grid.hpp"
#include "imview/component/opengl/point_cloud.hpp"
#include "imview/component/opengl/gl_scene_manager.hpp"

using namespace quickviz;

// Custom scene object that executes test operations in the main thread
class PointCloudTester : public SceneObject {
public:
  enum class TestState {
    Idle,
    Initialize,
    SetupTest,
    RunTest,
    Complete
  };

  struct TestConfig {
    PointCloud::BufferUpdateStrategy strategy;
    std::string strategy_name;
    size_t point_count;
    float point_size;
    size_t iterations;
  };

  struct TestResult {
    float avg_update_time_ms;
    float min_update_time_ms;
    float max_update_time_ms;
    std::vector<float> update_times_ms;
  };

  PointCloudTester(std::string name, PointCloud* point_cloud)
      : SceneObject(name), point_cloud_(point_cloud), state_(TestState::Idle) {}

  void QueueTest(const TestConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    test_queue_.push(config);
    if (state_ == TestState::Idle) {
      state_ = TestState::Initialize;
    }
  }

  bool IsTestComplete() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_ == TestState::Complete && test_queue_.empty();
  }

  std::vector<TestResult> GetResults() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return results_;
  }

  void OnRender() override {
    std::lock_guard<std::mutex> lock(mutex_);
    
    switch (state_) {
      case TestState::Initialize:
        if (!test_queue_.empty()) {
          current_config_ = test_queue_.front();
          test_queue_.pop();
          
          // Generate test points if needed
          if (test_points_.empty() || test_points_.size() != current_config_.point_count) {
            test_points_ = GenerateRandomPoints(current_config_.point_count, 5.0f);
            std::cout << "Generated " << test_points_.size() << " points for testing" << std::endl;
          }
          
          // Preallocate buffers
          point_cloud_->PreallocateBuffers(current_config_.point_count);
          point_cloud_->SetScalarRange(0.0f, 1.0f);
          
          // Initialize with points - use move semantics for efficiency
          point_cloud_->SetPoints(std::vector<glm::vec4>(test_points_), PointCloud::ColorMode::kScalarField);
          
          state_ = TestState::SetupTest;
        } else {
          state_ = TestState::Complete;
        }
        break;
        
      case TestState::SetupTest:
        // Set up for the current test
        point_cloud_->SetPointSize(current_config_.point_size);
        point_cloud_->SetBufferUpdateStrategy(current_config_.strategy);
        
        // Reset test data
        current_result_ = TestResult();
        current_result_.update_times_ms.reserve(current_config_.iterations);
        current_iteration_ = 0;
        
        std::cout << "\n=== Testing " << current_config_.strategy_name 
                  << " with " << current_config_.point_count << " points, size "
                  << current_config_.point_size << " ===" << std::endl;
        
        state_ = TestState::RunTest;
        break;
        
      case TestState::RunTest:
        if (current_iteration_ < current_config_.iterations) {
          // Measure the time for the update operation
          auto start_time = std::chrono::high_resolution_clock::now();
          
          // Use SetPoints instead of UpdatePoints
          point_cloud_->SetPoints(test_points_, PointCloud::ColorMode::kScalarField);
          
          auto end_time = std::chrono::high_resolution_clock::now();
          float duration_ms = std::chrono::duration<float, std::milli>(end_time - start_time).count();
          
          current_result_.update_times_ms.push_back(duration_ms);
          current_iteration_++;
          
          // Add a small delay between iterations to make the UI responsive
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } else {
          // Calculate statistics
          if (!current_result_.update_times_ms.empty()) {
            current_result_.min_update_time_ms = *std::min_element(
                current_result_.update_times_ms.begin(), current_result_.update_times_ms.end());
                
            current_result_.max_update_time_ms = *std::max_element(
                current_result_.update_times_ms.begin(), current_result_.update_times_ms.end());
                
            current_result_.avg_update_time_ms = std::accumulate(
                current_result_.update_times_ms.begin(),
                current_result_.update_times_ms.end(), 0.0f) / 
                current_result_.update_times_ms.size();
          }
          
          // Print results
          std::cout << std::fixed << std::setprecision(3);
          std::cout << "Strategy: " << current_config_.strategy_name << std::endl;
          std::cout << "  Avg: " << current_result_.avg_update_time_ms << " ms" << std::endl;
          std::cout << "  Min: " << current_result_.min_update_time_ms << " ms" << std::endl;
          std::cout << "  Max: " << current_result_.max_update_time_ms << " ms" << std::endl;
          
          // Store results
          results_.push_back(current_result_);
          
          // Move to next test
          state_ = TestState::Initialize;
        }
        break;
        
      case TestState::Complete:
        // All tests are complete
        break;
        
      case TestState::Idle:
        // Nothing to do
        break;
    }
  }

  void OnResize(float width, float height) override {
    // Nothing to do
  }

  void OnJoystickUpdate(const JoystickInput& input) override {
    // Nothing to do
  }

private:
  // Generate random points for testing
  std::vector<glm::vec4> GenerateRandomPoints(size_t count, float range) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(-range, range);

    std::vector<glm::vec4> points;
    points.reserve(count);

    for (size_t i = 0; i < count; ++i) {
      float x = dist(gen);
      float y = dist(gen);
      float z = dist(gen);

      // Ensure w is properly normalized and not NaN or Inf
      float w = 0.0f;
      float dist_squared = x * x + y * y + z * z;
      if (dist_squared > 0.0f) {
        w = std::sqrt(dist_squared) / range;
      }

      // Clamp w to valid range
      w = std::max(0.0f, std::min(1.0f, w));

      points.push_back(glm::vec4(x, y, z, w));
    }

    return points;
  }

  PointCloud* point_cloud_;
  mutable std::mutex mutex_;
  TestState state_;
  std::queue<TestConfig> test_queue_;
  TestConfig current_config_;
  TestResult current_result_;
  std::vector<TestResult> results_;
  std::vector<glm::vec4> test_points_;
  size_t current_iteration_ = 0;
};

int main(int argc, char* argv[]) {
  // Initialize viewer first to create OpenGL context
  Viewer viewer("Point Cloud Buffer Strategies Test", 1280, 720);

  // Create a box to manage size & position of the OpenGL scene
  auto box = std::make_shared<Box>("main_box");
  box->SetFlexDirection(Styling::FlexDirection::kRow);
  box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
  box->SetAlignItems(Styling::AlignItems::kStretch);

  // Create a OpenGL scene manager to manage the OpenGL objects
  auto gl_sm = std::make_shared<GlSceneManager>("OpenGL Scene");
  gl_sm->SetAutoLayout(true);
  gl_sm->SetNoTitleBar(true);
  gl_sm->SetFlexGrow(1.0f);
  gl_sm->SetFlexShrink(0.0f);

  // Create point cloud with preallocated buffers
  auto point_cloud = std::make_unique<PointCloud>();

  // Add a grid for reference
  auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));

  // Get a pointer to the point cloud before moving it
  PointCloud* point_cloud_ptr = point_cloud.get();

  // Add objects to scene manager
  gl_sm->AddOpenGLObject("point_cloud", std::move(point_cloud));
  gl_sm->AddOpenGLObject("grid", std::move(grid));

  // Add scene manager to box and add it to the viewer
  box->AddChild(gl_sm);
  viewer.AddSceneObject(box);

  // Create the point cloud tester
  auto tester = std::make_shared<PointCloudTester>("tester", point_cloud_ptr);
  viewer.AddSceneObject(tester);

  // Queue tests to run
  const size_t ITERATIONS = 10;
  const std::vector<size_t> POINT_COUNTS = {1000, 10000, 100000};
  const std::vector<float> POINT_SIZES = {1.0f, 5.0f};

  // Start a background thread to queue tests and monitor progress
  std::thread test_thread([&]() {
    // Wait a bit for the viewer to initialize
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Queue all the tests
    for (size_t point_count : POINT_COUNTS) {
      for (float point_size : POINT_SIZES) {
        // Test BufferSubData strategy
        tester->QueueTest({
          PointCloud::BufferUpdateStrategy::kBufferSubData,
          "BufferSubData",
          point_count,
          point_size,
          ITERATIONS
        });

        // Test MapBuffer strategy
        tester->QueueTest({
          PointCloud::BufferUpdateStrategy::kMapBuffer,
          "MapBuffer",
          point_count,
          point_size,
          ITERATIONS
        });

        // Test Auto strategy
        tester->QueueTest({
          PointCloud::BufferUpdateStrategy::kAuto,
          "Auto",
          point_count,
          point_size,
          ITERATIONS
        });
      }
    }

    // Wait for all tests to complete
    while (!tester->IsTestComplete()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Print summary of results
    auto results = tester->GetResults();
    std::cout << "\n=== Test Summary ===" << std::endl;
    
    // Close the viewer
    viewer.SetWindowShouldClose();
  });

  // Show the viewer - this will block until the window is closed
  viewer.Show();

  // Clean up
  if (test_thread.joinable()) {
    test_thread.join();
  }

  return 0;
} 