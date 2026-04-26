/*
 * @file test_enhanced_event_system.cpp
 * @date 8/30/25
 * @brief Tests for the modern unified event system
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "core/event/event_dispatcher.hpp"
#include "core/event/input_event.hpp"

// Mouse button constants
const int kLeft = 0;
const int kRight = 1; 
const int kMiddle = 2;

using namespace quickviz;

class ModernEventSystemTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dispatcher_ = std::make_unique<EventDispatcher>();
  }

  std::unique_ptr<EventDispatcher> dispatcher_;
};

// Test basic function handler registration
TEST_F(ModernEventSystemTest, FunctionHandlerRegistration) {
  bool handler_called = false;
  
  auto handler_func = [&handler_called](std::shared_ptr<BaseEvent> event) -> bool {
    handler_called = true;
    return false;  // Don't consume
  };
  
  dispatcher_->RegisterHandler("", handler_func, "test_handler", 0);
  
  auto event = std::make_shared<InputEvent>(InputEventType::kMousePress, kLeft);
  dispatcher_->DispatchEvent(event);
  
  EXPECT_TRUE(handler_called);
}

// Test priority ordering
TEST_F(ModernEventSystemTest, PriorityOrdering) {
  std::vector<int> call_order;
  
  // Register handlers with different priorities
  auto high_priority = [&call_order](std::shared_ptr<BaseEvent> event) -> bool {
    call_order.push_back(1);
    return false;
  };
  
  auto low_priority = [&call_order](std::shared_ptr<BaseEvent> event) -> bool {
    call_order.push_back(2);
    return false;
  };
  
  dispatcher_->RegisterHandler("", high_priority, "high", 100);
  dispatcher_->RegisterHandler("", low_priority, "low", 1);
  
  auto event = std::make_shared<InputEvent>(InputEventType::kMousePress);
  dispatcher_->DispatchEvent(event);
  
  ASSERT_EQ(call_order.size(), 2);
  EXPECT_EQ(call_order[0], 1);  // High priority first
  EXPECT_EQ(call_order[1], 2);  // Low priority second
}

// Test event consumption
TEST_F(ModernEventSystemTest, EventConsumption) {
  bool first_called = false;
  bool second_called = false;
  
  auto consuming_handler = [&first_called](std::shared_ptr<BaseEvent> event) -> bool {
    first_called = true;
    return true;  // Consume event
  };
  
  auto second_handler = [&second_called](std::shared_ptr<BaseEvent> event) -> bool {
    second_called = true;
    return false;
  };
  
  dispatcher_->RegisterHandler("", consuming_handler, "first", 100);
  dispatcher_->RegisterHandler("", second_handler, "second", 50);
  
  auto event = std::make_shared<InputEvent>(InputEventType::kMousePress);
  bool consumed = dispatcher_->DispatchEvent(event);
  
  EXPECT_TRUE(consumed);
  EXPECT_TRUE(first_called);
  EXPECT_FALSE(second_called);  // Should not be called due to consumption
}

// Test typed handler
TEST_F(ModernEventSystemTest, TypedHandler) {
  bool input_handler_called = false;
  
  dispatcher_->RegisterTypedHandler<InputEvent>("typed_handler",
    [&input_handler_called](std::shared_ptr<InputEvent> event) -> bool {
      input_handler_called = true;
      EXPECT_EQ(event->GetType(), InputEventType::kMousePress);
      return false;
    });
  
  // This should trigger the typed handler
  auto input_event = std::make_shared<InputEvent>(InputEventType::kMousePress);
  dispatcher_->DispatchEvent(input_event);
  EXPECT_TRUE(input_handler_called);
  
  // Reset for next test
  input_handler_called = false;
  
  // This should NOT trigger the typed handler (wrong type)
  auto generic_event = std::make_shared<Event<int>>(EventSource::kCustomEvent, "test", 42);
  dispatcher_->DispatchEvent(generic_event);
  EXPECT_FALSE(input_handler_called);
}

// Test handler management
TEST_F(ModernEventSystemTest, HandlerManagement) {
  bool handler_called = false;
  
  auto handler_func = [&handler_called](std::shared_ptr<BaseEvent> event) -> bool {
    handler_called = true;
    return false;
  };
  
  // Register handler
  dispatcher_->RegisterHandler("", handler_func, "test_handler", 0);
  EXPECT_EQ(dispatcher_->GetHandlerCount(), 1);
  
  // Get handler
  auto handler = dispatcher_->GetHandler("test_handler");
  EXPECT_NE(handler, nullptr);
  EXPECT_EQ(handler->GetName(), "test_handler");
  
  // Test enable/disable
  handler->SetEnabled(false);
  auto event = std::make_shared<InputEvent>(InputEventType::kMousePress);
  dispatcher_->DispatchEvent(event);
  EXPECT_FALSE(handler_called);
  
  handler->SetEnabled(true);
  dispatcher_->DispatchEvent(event);
  EXPECT_TRUE(handler_called);
  
  // Unregister handler
  dispatcher_->UnregisterHandler("test_handler");
  EXPECT_EQ(dispatcher_->GetHandlerCount(), 0);
}

// Test global enable/disable
TEST_F(ModernEventSystemTest, GlobalEnableDisable) {
  bool handler_called = false;
  
  auto handler_func = [&handler_called](std::shared_ptr<BaseEvent> event) -> bool {
    handler_called = true;
    return false;
  };
  
  dispatcher_->RegisterHandler("", handler_func, "test_handler", 0);
  
  // Disable globally
  dispatcher_->SetEnabled(false);
  auto event = std::make_shared<InputEvent>(InputEventType::kMousePress);
  dispatcher_->DispatchEvent(event);
  EXPECT_FALSE(handler_called);
  
  // Re-enable globally
  dispatcher_->SetEnabled(true);
  dispatcher_->DispatchEvent(event);
  EXPECT_TRUE(handler_called);
}

// Test InputEvent handling with EventDispatcher
TEST_F(ModernEventSystemTest, InputEventHandling) {
  bool input_handler_called = false;
  InputEventType received_type;
  
  // Register a typed handler for InputEvent
  dispatcher_->RegisterTypedHandler<InputEvent>("input_handler",
    [&input_handler_called, &received_type](std::shared_ptr<InputEvent> event) -> bool {
      input_handler_called = true;
      received_type = event->GetType();
      return false;
    });
  
  // Create and dispatch an InputEvent
  auto input_event = std::make_shared<InputEvent>(InputEventType::kMousePress, kLeft);
  bool consumed = dispatcher_->DispatchEvent(input_event);
  
  EXPECT_FALSE(consumed);
  EXPECT_TRUE(input_handler_called);
  EXPECT_EQ(received_type, InputEventType::kMousePress);
}

// Test input event filtering with generic handlers
TEST_F(ModernEventSystemTest, InputEventFiltering) {
  int mouse_count = 0;
  int keyboard_count = 0;
  int total_count = 0;
  
  // Generic handler that counts all events
  dispatcher_->RegisterHandler("", 
    [&total_count](std::shared_ptr<BaseEvent> event) -> bool {
      total_count++;
      return false;
    }, "total_counter", 0);
  
  // Specialized handler for mouse events
  dispatcher_->RegisterHandler("",
    [&mouse_count](std::shared_ptr<BaseEvent> event) -> bool {
      auto input_event = std::dynamic_pointer_cast<InputEvent>(event);
      if (input_event && input_event->IsMouseEvent()) {
        mouse_count++;
      }
      return false;
    }, "mouse_counter", 0);
    
  // Specialized handler for keyboard events  
  dispatcher_->RegisterHandler("",
    [&keyboard_count](std::shared_ptr<BaseEvent> event) -> bool {
      auto input_event = std::dynamic_pointer_cast<InputEvent>(event);
      if (input_event && input_event->IsKeyboardEvent()) {
        keyboard_count++;
      }
      return false;
    }, "keyboard_counter", 0);
  
  // Send mouse event
  auto mouse_event = std::make_shared<InputEvent>(InputEventType::kMousePress, kLeft);
  dispatcher_->DispatchEvent(mouse_event);
  
  // Send keyboard event
  auto key_event = std::make_shared<InputEvent>(InputEventType::kKeyPress, 65);
  dispatcher_->DispatchEvent(key_event);
  
  EXPECT_EQ(total_count, 2);     // Both events counted
  EXPECT_EQ(mouse_count, 1);     // Only mouse event counted
  EXPECT_EQ(keyboard_count, 1);  // Only keyboard event counted
}