/*
 * utest_event_system.cpp
 *
 * Created on: August 27, 2025
 * Description: Unit tests for virtual scene event system
 * 
 * Tests the EventDispatcher, EventBuilder, and event integration with VirtualScene.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "vscene/event_system.hpp"
#include "vscene/virtual_scene.hpp"
#include "vscene/virtual_sphere.hpp"
#include "vscene/mock_render_backend.hpp"

using namespace quickviz;

class EventSystemTest : public ::testing::Test {
protected:
    void SetUp() override {
        dispatcher_ = std::make_unique<EventDispatcher>();
        events_received_.clear();
        
        // Set up event handler that captures all events
        event_handler_ = [this](const VirtualEvent& event) {
            events_received_.push_back(event);
        };
    }

    std::unique_ptr<EventDispatcher> dispatcher_;
    std::vector<VirtualEvent> events_received_;
    EventDispatcher::EventHandler event_handler_;
};

// Test basic event subscription and dispatch
TEST_F(EventSystemTest, BasicSubscriptionAndDispatch) {
    // Subscribe to ObjectClicked events
    dispatcher_->Subscribe(VirtualEventType::ObjectClicked, event_handler_);
    
    EXPECT_EQ(dispatcher_->GetSubscriberCount(VirtualEventType::ObjectClicked), 1);
    EXPECT_EQ(dispatcher_->GetTotalSubscribers(), 1);
    
    // Create and dispatch an event
    auto event = EventBuilder::ObjectClicked("test_sphere", glm::vec2(100, 200), glm::vec3(1, 2, 3), 0);
    dispatcher_->Dispatch(event);
    
    // Verify event was received
    ASSERT_EQ(events_received_.size(), 1);
    EXPECT_EQ(events_received_[0].type, VirtualEventType::ObjectClicked);
    EXPECT_EQ(events_received_[0].object_id, "test_sphere");
    EXPECT_EQ(events_received_[0].screen_pos, glm::vec2(100, 200));
    EXPECT_EQ(events_received_[0].world_pos, glm::vec3(1, 2, 3));
    EXPECT_EQ(events_received_[0].mouse_button, 0);
}

// Test event filtering
TEST_F(EventSystemTest, EventFiltering) {
    // Create filter that only allows events for "sphere1"
    auto filter = [](const VirtualEvent& event) {
        return event.object_id == "sphere1";
    };
    
    dispatcher_->Subscribe(VirtualEventType::ObjectClicked, event_handler_, filter);
    
    // Dispatch events for different objects
    dispatcher_->Dispatch(EventBuilder::ObjectClicked("sphere1", glm::vec2(0, 0), glm::vec3(0, 0, 0)));
    dispatcher_->Dispatch(EventBuilder::ObjectClicked("sphere2", glm::vec2(0, 0), glm::vec3(0, 0, 0)));
    dispatcher_->Dispatch(EventBuilder::ObjectClicked("sphere1", glm::vec2(0, 0), glm::vec3(0, 0, 0)));
    
    // Should only receive events for sphere1
    EXPECT_EQ(events_received_.size(), 2);
    for (const auto& event : events_received_) {
        EXPECT_EQ(event.object_id, "sphere1");
    }
}

// Test multiple subscribers
TEST_F(EventSystemTest, MultipleSubscribers) {
    std::vector<VirtualEvent> events_handler1;
    std::vector<VirtualEvent> events_handler2;
    
    auto handler1 = [&events_handler1](const VirtualEvent& event) {
        events_handler1.push_back(event);
    };
    
    auto handler2 = [&events_handler2](const VirtualEvent& event) {
        events_handler2.push_back(event);
    };
    
    dispatcher_->Subscribe(VirtualEventType::ObjectClicked, handler1);
    dispatcher_->Subscribe(VirtualEventType::ObjectClicked, handler2);
    
    EXPECT_EQ(dispatcher_->GetSubscriberCount(VirtualEventType::ObjectClicked), 2);
    
    // Dispatch event
    dispatcher_->Dispatch(EventBuilder::ObjectClicked("test", glm::vec2(0, 0), glm::vec3(0, 0, 0)));
    
    // Both handlers should receive the event
    EXPECT_EQ(events_handler1.size(), 1);
    EXPECT_EQ(events_handler2.size(), 1);
}

// Test event batching
TEST_F(EventSystemTest, EventBatching) {
    dispatcher_->Subscribe(VirtualEventType::ObjectClicked, event_handler_);
    
    // Start batching
    dispatcher_->BeginBatch();
    
    // Dispatch events while batching
    dispatcher_->Dispatch(EventBuilder::ObjectClicked("sphere1", glm::vec2(0, 0), glm::vec3(0, 0, 0)));
    dispatcher_->Dispatch(EventBuilder::ObjectClicked("sphere2", glm::vec2(0, 0), glm::vec3(0, 0, 0)));
    
    // Should not have received events yet
    EXPECT_EQ(events_received_.size(), 0);
    
    // End batching - should receive all batched events
    dispatcher_->EndBatch();
    
    EXPECT_EQ(events_received_.size(), 2);
    EXPECT_EQ(events_received_[0].object_id, "sphere1");
    EXPECT_EQ(events_received_[1].object_id, "sphere2");
}

// Test unsubscribe
TEST_F(EventSystemTest, Unsubscribe) {
    dispatcher_->Subscribe(VirtualEventType::ObjectClicked, event_handler_);
    
    // Dispatch event - should be received
    dispatcher_->Dispatch(EventBuilder::ObjectClicked("test", glm::vec2(0, 0), glm::vec3(0, 0, 0)));
    EXPECT_EQ(events_received_.size(), 1);
    
    // Unsubscribe and dispatch another event
    dispatcher_->Unsubscribe(VirtualEventType::ObjectClicked);
    events_received_.clear();
    dispatcher_->Dispatch(EventBuilder::ObjectClicked("test2", glm::vec2(0, 0), glm::vec3(0, 0, 0)));
    
    // Should not receive the second event
    EXPECT_EQ(events_received_.size(), 0);
    EXPECT_EQ(dispatcher_->GetSubscriberCount(VirtualEventType::ObjectClicked), 0);
}

// Test EventBuilder functionality
TEST_F(EventSystemTest, EventBuilders) {
    // Test ObjectClicked builder
    auto click_event = EventBuilder::ObjectClicked("obj1", glm::vec2(10, 20), glm::vec3(1, 2, 3), 1);
    EXPECT_EQ(click_event.type, VirtualEventType::ObjectClicked);
    EXPECT_EQ(click_event.object_id, "obj1");
    EXPECT_EQ(click_event.screen_pos, glm::vec2(10, 20));
    EXPECT_EQ(click_event.world_pos, glm::vec3(1, 2, 3));
    EXPECT_EQ(click_event.mouse_button, 1);
    EXPECT_GT(click_event.timestamp, 0);
    
    // Test ObjectDragged builder
    auto drag_event = EventBuilder::ObjectDragged("obj2", glm::vec2(30, 40), glm::vec3(0.5, -0.5, 0));
    EXPECT_EQ(drag_event.type, VirtualEventType::ObjectDragged);
    EXPECT_EQ(drag_event.object_id, "obj2");
    EXPECT_EQ(drag_event.screen_pos, glm::vec2(30, 40));
    EXPECT_EQ(drag_event.world_pos, glm::vec3(0.5, -0.5, 0)); // world_pos used for delta
    
    // Test SelectionChanged builder
    std::vector<std::string> selected_ids = {"obj1", "obj2", "obj3"};
    auto selection_event = EventBuilder::SelectionChanged(selected_ids);
    EXPECT_EQ(selection_event.type, VirtualEventType::SelectionChanged);
    EXPECT_EQ(selection_event.object_id, "obj1"); // Primary selection
    EXPECT_EQ(selection_event.object_ids, selected_ids);
    
    // Test BackgroundClicked builder
    auto bg_event = EventBuilder::BackgroundClicked(glm::vec2(50, 60), glm::vec3(5, 6, 7), 2);
    EXPECT_EQ(bg_event.type, VirtualEventType::BackgroundClicked);
    EXPECT_EQ(bg_event.object_id, ""); // No object for background events
    EXPECT_EQ(bg_event.screen_pos, glm::vec2(50, 60));
    EXPECT_EQ(bg_event.world_pos, glm::vec3(5, 6, 7));
    EXPECT_EQ(bg_event.mouse_button, 2);
}

// Test VirtualScene integration with events
class VirtualSceneEventTest : public ::testing::Test {
protected:
    void SetUp() override {
        scene_ = std::make_unique<VirtualScene>();
        backend_ = std::make_unique<MockRenderBackend>();
        scene_->SetRenderBackend(std::move(backend_));
        
        events_received_.clear();
        
        // Subscribe to all event types we care about
        auto* dispatcher = scene_->GetEventDispatcher();
        
        event_handler_ = [this](const VirtualEvent& event) {
            events_received_.push_back(event);
        };
        
        dispatcher->Subscribe(VirtualEventType::ObjectAdded, event_handler_);
        dispatcher->Subscribe(VirtualEventType::ObjectRemoved, event_handler_);
        dispatcher->Subscribe(VirtualEventType::SelectionChanged, event_handler_);
        dispatcher->Subscribe(VirtualEventType::SelectionCleared, event_handler_);
    }

    std::unique_ptr<VirtualScene> scene_;
    std::unique_ptr<MockRenderBackend> backend_;
    std::vector<VirtualEvent> events_received_;
    EventDispatcher::EventHandler event_handler_;
};

// Test object addition events
TEST_F(VirtualSceneEventTest, ObjectAdditionEvents) {
    auto sphere1 = std::make_unique<VirtualSphere>("sphere1", 1.0f);
    scene_->AddObject("sphere1", std::move(sphere1));
    
    // Should have received ObjectAdded event
    ASSERT_EQ(events_received_.size(), 1);
    EXPECT_EQ(events_received_[0].type, VirtualEventType::ObjectAdded);
    EXPECT_EQ(events_received_[0].object_id, "sphere1");
}

// Test object removal events
TEST_F(VirtualSceneEventTest, ObjectRemovalEvents) {
    // Add object first
    auto sphere1 = std::make_unique<VirtualSphere>("sphere1", 1.0f);
    scene_->AddObject("sphere1", std::move(sphere1));
    events_received_.clear(); // Clear the ObjectAdded event
    
    // Remove object
    scene_->RemoveObject("sphere1");
    
    // Should have received ObjectRemoved event
    ASSERT_EQ(events_received_.size(), 1);
    EXPECT_EQ(events_received_[0].type, VirtualEventType::ObjectRemoved);
    EXPECT_EQ(events_received_[0].object_id, "sphere1");
}

// Test selection change events
TEST_F(VirtualSceneEventTest, SelectionChangeEvents) {
    // Add objects
    auto sphere1 = std::make_unique<VirtualSphere>("sphere1", 1.0f);
    auto sphere2 = std::make_unique<VirtualSphere>("sphere2", 1.0f);
    scene_->AddObject("sphere1", std::move(sphere1));
    scene_->AddObject("sphere2", std::move(sphere2));
    events_received_.clear(); // Clear ObjectAdded events
    
    // Select object
    scene_->SetSelected("sphere1", true);
    
    // Should have received SelectionChanged event
    ASSERT_EQ(events_received_.size(), 1);
    EXPECT_EQ(events_received_[0].type, VirtualEventType::SelectionChanged);
    EXPECT_EQ(events_received_[0].object_id, "sphere1");
    ASSERT_EQ(events_received_[0].object_ids.size(), 1);
    EXPECT_EQ(events_received_[0].object_ids[0], "sphere1");
    
    events_received_.clear();
    
    // Add second object to selection
    scene_->SetSelected("sphere2", true);
    
    // Should have received another SelectionChanged event
    ASSERT_EQ(events_received_.size(), 1);
    EXPECT_EQ(events_received_[0].type, VirtualEventType::SelectionChanged);
    EXPECT_EQ(events_received_[0].object_ids.size(), 2);
    // Check that both objects are in selection (order might vary)
    EXPECT_TRUE(std::find(events_received_[0].object_ids.begin(), events_received_[0].object_ids.end(), "sphere1") != events_received_[0].object_ids.end());
    EXPECT_TRUE(std::find(events_received_[0].object_ids.begin(), events_received_[0].object_ids.end(), "sphere2") != events_received_[0].object_ids.end());
}

// Test selection cleared events
TEST_F(VirtualSceneEventTest, SelectionClearedEvents) {
    // Add and select objects
    auto sphere1 = std::make_unique<VirtualSphere>("sphere1", 1.0f);
    scene_->AddObject("sphere1", std::move(sphere1));
    scene_->SetSelected("sphere1", true);
    events_received_.clear(); // Clear previous events
    
    // Clear selection
    scene_->ClearSelection();
    
    // Should have received SelectionCleared event
    ASSERT_EQ(events_received_.size(), 1);
    EXPECT_EQ(events_received_[0].type, VirtualEventType::SelectionCleared);
}

// Test object removal with selection events
TEST_F(VirtualSceneEventTest, ObjectRemovalWithSelection) {
    // Add and select object
    auto sphere1 = std::make_unique<VirtualSphere>("sphere1", 1.0f);
    scene_->AddObject("sphere1", std::move(sphere1));
    scene_->SetSelected("sphere1", true);
    events_received_.clear(); // Clear previous events
    
    // Remove selected object
    scene_->RemoveObject("sphere1");
    
    // Should have received both ObjectRemoved and SelectionChanged events
    EXPECT_EQ(events_received_.size(), 2);
    
    // Find events by type
    VirtualEvent* removed_event = nullptr;
    VirtualEvent* selection_event = nullptr;
    
    for (auto& event : events_received_) {
        if (event.type == VirtualEventType::ObjectRemoved) {
            removed_event = &event;
        } else if (event.type == VirtualEventType::SelectionChanged) {
            selection_event = &event;
        }
    }
    
    ASSERT_NE(removed_event, nullptr);
    ASSERT_NE(selection_event, nullptr);
    
    EXPECT_EQ(removed_event->object_id, "sphere1");
    EXPECT_EQ(selection_event->object_ids.size(), 0); // Selection should be empty
}

// Test no events for no-op operations
TEST_F(VirtualSceneEventTest, NoEventsForNoOps) {
    // Try to select non-existent object
    scene_->SetSelected("non_existent", true);
    EXPECT_EQ(events_received_.size(), 0);
    
    // Try to clear empty selection
    scene_->ClearSelection();
    EXPECT_EQ(events_received_.size(), 0);
    
    // Try to remove non-existent object
    scene_->RemoveObject("non_existent");
    EXPECT_EQ(events_received_.size(), 0);
}