/*
 * @file test_input_event.cpp
 * @date 8/30/25
 * @brief Unit tests for InputEvent, InputMapping, and ModifierKeys
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "core/event/input_event.hpp"
#include "core/event/input_mapping.hpp"

// Mouse button constants (avoiding enum conflict)
const int kLeft = 0;
const int kRight = 1;
const int kMiddle = 2;

using namespace quickviz;

// InputEvent tests
class InputEventTest : public ::testing::Test {};

TEST_F(InputEventTest, InputEventConstruction) {
  InputEvent event(InputEventType::kMousePress, kLeft);

  EXPECT_EQ(event.GetType(), InputEventType::kMousePress);
  EXPECT_EQ(event.GetButtonOrKey(), kLeft);
  EXPECT_EQ(event.GetMouseButton(), kLeft);
  EXPECT_FALSE(event.IsConsumed());
  EXPECT_TRUE(event.IsMouseEvent());
  EXPECT_FALSE(event.IsKeyboardEvent());
}

TEST_F(InputEventTest, InputEventModifiers) {
  InputEvent event(InputEventType::kKeyPress, 65);  // 'A' key

  ModifierKeys mods;
  mods.ctrl = true;
  mods.shift = true;
  event.SetModifiers(mods);

  EXPECT_EQ(event.GetKey(), 65);
  EXPECT_TRUE(event.GetModifiers().ctrl);
  EXPECT_TRUE(event.GetModifiers().shift);
  EXPECT_FALSE(event.GetModifiers().alt);
  EXPECT_TRUE(event.IsKeyboardEvent());
  EXPECT_FALSE(event.IsMouseEvent());
}

TEST_F(InputEventTest, InputEventPositionAndDelta) {
  InputEvent event(InputEventType::kMouseMove);
  
  event.SetScreenPosition(glm::vec2(100, 200));
  event.SetDelta(glm::vec2(10, -5));
  
  EXPECT_EQ(event.GetScreenPosition().x, 100);
  EXPECT_EQ(event.GetScreenPosition().y, 200);
  EXPECT_EQ(event.GetDelta().x, 10);
  EXPECT_EQ(event.GetDelta().y, -5);
}

TEST_F(InputEventTest, InputEventConsumption) {
  InputEvent event(InputEventType::kMousePress);
  
  EXPECT_FALSE(event.IsConsumed());
  
  event.Consume();
  EXPECT_TRUE(event.IsConsumed());
}

// InputMapping tests
class InputMappingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    mapping_ = std::make_unique<InputMapping>();
  }

  std::unique_ptr<InputMapping> mapping_;
};

TEST_F(InputMappingTest, DefaultMappings) {
  // Test some default mappings exist
  InputEvent left_click(InputEventType::kMousePress, 0); // Left mouse button
  EXPECT_TRUE(mapping_->IsActionTriggered("select_single", left_click));
  
  InputEvent right_click(InputEventType::kMousePress, 1); // Right mouse button  
  EXPECT_TRUE(mapping_->IsActionTriggered("camera_rotate", right_click));
}

TEST_F(InputMappingTest, ModifierMappings) {
  ModifierKeys ctrl_shift;
  ctrl_shift.ctrl = true;
  ctrl_shift.shift = true;
  
  InputEvent event(InputEventType::kKeyPress, 90); // Z key (redo = Ctrl+Shift+Z)
  event.SetModifiers(ctrl_shift);
  
  EXPECT_TRUE(mapping_->IsActionTriggered("redo", event));
}

TEST_F(InputMappingTest, CustomMappings) {
  mapping_->MapKeyAction("custom_action", 72); // H key
  
  InputEvent event(InputEventType::kKeyPress, 72);
  EXPECT_TRUE(mapping_->IsActionTriggered("custom_action", event));
}

TEST_F(InputMappingTest, MultipleBindingsPerAction) {
  mapping_->MapKeyAction("jump", 32); // Spacebar
  mapping_->MapKeyAction("jump", 74); // J key
  
  InputEvent space_event(InputEventType::kKeyPress, 32);
  InputEvent j_event(InputEventType::kKeyPress, 74);
  
  EXPECT_TRUE(mapping_->IsActionTriggered("jump", space_event));
  EXPECT_TRUE(mapping_->IsActionTriggered("jump", j_event));
}

TEST_F(InputMappingTest, SaveLoadMappings) {
  mapping_->MapKeyAction("test_action", 84); // T key
  
  // For this test, just verify the mapping works
  InputEvent event(InputEventType::kKeyPress, 84);
  EXPECT_TRUE(mapping_->IsActionTriggered("test_action", event));
  
  // Test save/load via file
  std::string temp_file = "/tmp/test_mapping.cfg";
  mapping_->SaveToFile(temp_file);
  
  auto new_mapping = std::make_unique<InputMapping>();
  new_mapping->LoadFromFile(temp_file);
  
  EXPECT_TRUE(new_mapping->IsActionTriggered("test_action", event));
}

// ModifierKeys tests
class ModifierKeysTest : public ::testing::Test {};

TEST_F(ModifierKeysTest, Equality) {
  ModifierKeys mods1;
  mods1.ctrl = true;
  mods1.alt = true;
  
  ModifierKeys mods2;
  mods2.ctrl = true;
  mods2.alt = true;
  
  ModifierKeys mods3;
  mods3.alt = true;
  
  EXPECT_EQ(mods1, mods2);
  EXPECT_NE(mods1, mods3);
}

TEST_F(ModifierKeysTest, IsEmpty) {
  ModifierKeys empty_mods;
  EXPECT_TRUE(empty_mods.IsEmpty());
  
  ModifierKeys ctrl_mods;
  ctrl_mods.ctrl = true;
  EXPECT_FALSE(ctrl_mods.IsEmpty());
}