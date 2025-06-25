/*
 * @file test_imview_integration.cpp
 * @date 2024-06-25
 * @brief Integration tests for ImView GUI components
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <memory>

#include "imview/viewer.hpp"
#include "imview/panel.hpp"
#include "imview/box.hpp"
#include "imview/scene_object.hpp"
#include "imview/styling.hpp"

using namespace quickviz;

// Test panel implementation
class TestPanel : public Panel {
public:
    TestPanel(const std::string& name) : Panel(name) {}
    
protected:
    void Draw() override {
        // Simple test implementation
        ImGui::Text("Test Panel: %s", GetName().c_str());
    }
};

class ImViewIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create viewer for testing  
        viewer_ = std::make_unique<Viewer>("Integration Test", 800, 600);
    }

    void TearDown() override {
        viewer_.reset();
    }

    std::unique_ptr<Viewer> viewer_;
};

TEST_F(ImViewIntegrationTest, CanCreateViewer) {
    ASSERT_NE(viewer_, nullptr);
    EXPECT_FALSE(viewer_->ShouldClose());
}

TEST_F(ImViewIntegrationTest, CanAddPanelToViewer) {
    auto panel = std::make_shared<TestPanel>("TestPanel");
    panel->SetAutoLayout(true);
    
    bool added = viewer_->AddSceneObject(panel);
    EXPECT_TRUE(added);
}

TEST_F(ImViewIntegrationTest, CanCreateNestedBoxLayout) {
    auto root_box = std::make_shared<Box>("RootBox");
    auto child_box1 = std::make_shared<Box>("ChildBox1");
    auto child_box2 = std::make_shared<Box>("ChildBox2");
    
    // Create nested structure
    root_box->AddChild(child_box1);
    root_box->AddChild(child_box2);
    
    // Add panels to child boxes
    auto panel1 = std::make_shared<TestPanel>("Panel1");
    auto panel2 = std::make_shared<TestPanel>("Panel2");
    
    child_box1->AddChild(panel1);
    child_box2->AddChild(panel2);
    
    // Add root to viewer
    bool added = viewer_->AddSceneObject(root_box);
    EXPECT_TRUE(added);
}

#ifdef ENABLE_AUTO_LAYOUT
TEST_F(ImViewIntegrationTest, AutoLayoutConfiguration) {
    auto box = std::make_shared<Box>("LayoutTestBox");
    
    // Test layout configuration using Styling namespace
    box->SetFlexDirection(Styling::FlexDirection::kColumn);
    box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
    box->SetAlignItems(Styling::AlignItems::kStretch);
    
    auto panel = std::make_shared<TestPanel>("LayoutTestPanel");
    panel->SetAutoLayout(true);
    
    box->AddChild(panel);
    viewer_->AddSceneObject(box);
    
    SUCCEED();
}
#endif

// Test scene object lifecycle
TEST_F(ImViewIntegrationTest, SceneObjectLifecycle) {
    auto panel = std::make_shared<TestPanel>("LifecycleTest");
    panel->SetVisibility(true);
    
    EXPECT_TRUE(panel->IsVisible());
    EXPECT_EQ(panel->GetName(), "LifecycleTest");
    
    viewer_->AddSceneObject(panel);
    
    // Test visibility toggle
    panel->SetVisibility(false);
    EXPECT_FALSE(panel->IsVisible());
    
    panel->SetVisibility(true);
    EXPECT_TRUE(panel->IsVisible());
}