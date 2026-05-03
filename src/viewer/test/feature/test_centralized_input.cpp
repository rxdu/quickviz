/*
 * @file test_centralized_input.cpp
 * @date 9/1/25
 * @brief Test centralized input management functionality
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <memory>
#include <iostream>

#include "viewer/window.hpp"
#include "viewer/panel.hpp"
#include "viewer/input/gamepad_manager.hpp"
#include "core/event/input_event.hpp"

using namespace quickviz;

// Test panel that tracks input events
class TestPanel : public Panel {
 public:
  TestPanel(const std::string& name, int priority = 0) 
      : Panel(name), priority_(priority) {}

  void Draw() override {
    ImGui::Text("Test Panel: %s", GetName().c_str());
  }

  bool OnInputEvent(const InputEvent& event) override {
    received_events_.push_back(event);
    return consume_events_; // Return true to consume, false to pass through
  }

  int GetPriority() const override { return priority_; }

  void SetConsumeEvents(bool consume) { consume_events_ = consume; }
  const std::vector<InputEvent>& GetReceivedEvents() const { return received_events_; }
  void ClearEvents() { received_events_.clear(); }

 private:
  int priority_;
  bool consume_events_ = false;
  std::vector<InputEvent> received_events_;
};

class CentralizedInputTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create window with OpenGL context (required for ImGui)
    // Note: This test requires a display connection
    if (glfwInit()) {
      glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE); // Hidden window for testing
      window_ = std::make_unique<Window>("Test Window", 800, 600);
    } else {
      GTEST_SKIP() << "GLFW initialization failed - skipping GUI tests";
    }
  }

  void TearDown() override {
    // Window destructor automatically handles GamepadManager shutdown and glfwTerminate()
    window_.reset();
  }

  std::unique_ptr<Window> window_;
};

TEST_F(CentralizedInputTest, PanelRegistration) {
  if (!window_) return;

  // Create test panels
  auto panel1 = std::make_shared<TestPanel>("Panel1", 100);
  auto panel2 = std::make_shared<TestPanel>("Panel2", 50);

  // Register panels with window
  window_->RegisterPanel(panel1);
  window_->RegisterPanel(panel2);

  // Verify panels are registered
  EXPECT_EQ(window_->GetInputManager().GetHandlerCount(), 2);

  // Unregister one panel
  window_->UnregisterPanel("Panel1");
  EXPECT_EQ(window_->GetInputManager().GetHandlerCount(), 1);

  // Clear all panels
  window_->ClearPanels();
  EXPECT_EQ(window_->GetInputManager().GetHandlerCount(), 0);
}

TEST_F(CentralizedInputTest, PanelPriority) {
  if (!window_) return;

  // Create panels with different priorities
  auto high_priority = std::make_shared<TestPanel>("HighPriority", 100);
  auto low_priority = std::make_shared<TestPanel>("LowPriority", 50);

  // High priority panel consumes events
  high_priority->SetConsumeEvents(true);

  // Register panels (order shouldn't matter - priority should determine processing)
  window_->RegisterPanel(low_priority);
  window_->RegisterPanel(high_priority);

  // Create a test input event
  InputEvent test_event(InputEventType::kMousePress, 0); // Left mouse button
  test_event.SetScreenPosition(glm::vec2(100, 100));

  // Process the event
  bool consumed = window_->GetInputManager().ProcessEvent(test_event);

  // Verify event was consumed by high priority panel
  EXPECT_TRUE(consumed);
  EXPECT_EQ(high_priority->GetReceivedEvents().size(), 1);
  EXPECT_EQ(low_priority->GetReceivedEvents().size(), 0); // Should not receive due to consumption
}

TEST_F(CentralizedInputTest, EventPropagation) {
  if (!window_) return;

  // Create panels that don't consume events
  auto panel1 = std::make_shared<TestPanel>("Panel1", 100);
  auto panel2 = std::make_shared<TestPanel>("Panel2", 50);

  panel1->SetConsumeEvents(false); // Let events pass through
  panel2->SetConsumeEvents(false);

  window_->RegisterPanel(panel1);
  window_->RegisterPanel(panel2);

  // Create a test input event
  InputEvent test_event(InputEventType::kKeyPress, 65); // 'A' key

  // Process the event
  bool consumed = window_->GetInputManager().ProcessEvent(test_event);

  // Event should not be consumed but both panels should receive it
  EXPECT_FALSE(consumed);
  EXPECT_EQ(panel1->GetReceivedEvents().size(), 1);
  EXPECT_EQ(panel2->GetReceivedEvents().size(), 1);
}

TEST_F(CentralizedInputTest, PanelAttachment) {
  if (!window_) return;

  auto panel = std::make_shared<TestPanel>("TestPanel", 100);

  // Initially not attached
  EXPECT_FALSE(panel->IsAttachedToWindow());

  // Register with window (simulates attachment)
  window_->RegisterPanel(panel);
  // Note: AttachToWindow would need to be called from application code
  // This is a limitation of the current design - panel attachment is manual

  // For now, verify the input manager has the panel registered
  EXPECT_EQ(window_->GetInputManager().GetHandlerCount(), 1);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}