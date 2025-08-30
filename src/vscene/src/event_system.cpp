/*
 * @file event_system.cpp
 * @date August 27, 2025
 * @brief Implementation of virtual scene event system
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "vscene/event_system.hpp"
#include <chrono>
#include <algorithm>

namespace quickviz {

// ============================================================================
// VirtualEventDispatcher Implementation
// ============================================================================

void VirtualEventDispatcher::Subscribe(VirtualEventType type, EventHandler handler) {
    Subscribe(type, handler, nullptr);
}

void VirtualEventDispatcher::Subscribe(VirtualEventType type, EventHandler handler, EventFilter filter) {
    if (!handler) return;
    
    // Create empty filter if none provided
    EventFilter actual_filter = filter ? filter : [](const VirtualEvent&) { return true; };
    
    subscribers_[type].emplace_back(handler, actual_filter);
}

void VirtualEventDispatcher::Unsubscribe(VirtualEventType type) {
    subscribers_[type].clear();
}

void VirtualEventDispatcher::UnsubscribeAll() {
    subscribers_.clear();
}

void VirtualEventDispatcher::Dispatch(const VirtualEvent& event) {
    if (batching_enabled_) {
        batched_events_.push_back(event);
        return;
    }
    
    DispatchToSubscribers(event);
    events_dispatched_++;
}

void VirtualEventDispatcher::DispatchAsync(const VirtualEvent& event) {
    // For now, just dispatch synchronously
    // In a more advanced implementation, this could use a thread pool
    Dispatch(event);
}

void VirtualEventDispatcher::BeginBatch() {
    batching_enabled_ = true;
    batched_events_.clear();
}

void VirtualEventDispatcher::EndBatch() {
    batching_enabled_ = false;
    
    // Dispatch all batched events
    for (const auto& event : batched_events_) {
        DispatchToSubscribers(event);
        events_dispatched_++;
    }
    batched_events_.clear();
}

void VirtualEventDispatcher::ClearBatch() {
    batched_events_.clear();
}

size_t VirtualEventDispatcher::GetSubscriberCount(VirtualEventType type) const {
    auto it = subscribers_.find(type);
    return (it != subscribers_.end()) ? it->second.size() : 0;
}

size_t VirtualEventDispatcher::GetTotalSubscribers() const {
    size_t total = 0;
    for (const auto& [type, handlers] : subscribers_) {
        total += handlers.size();
    }
    return total;
}

void VirtualEventDispatcher::DispatchToSubscribers(const VirtualEvent& event) {
    auto it = subscribers_.find(event.type);
    if (it == subscribers_.end()) {
        return; // No subscribers for this event type
    }
    
    for (const auto& [handler, filter] : it->second) {
        if (PassesFilter(event, filter)) {
            try {
                handler(event);
            } catch (...) {
                // Silently continue if handler throws
                // In a production system, might want to log this
            }
        }
    }
}

bool VirtualEventDispatcher::PassesFilter(const VirtualEvent& event, const EventFilter& filter) const {
    if (!filter) return true;
    
    try {
        return filter(event);
    } catch (...) {
        return false; // If filter throws, consider it failed
    }
}

// ============================================================================
// EventBuilder Implementation  
// ============================================================================

VirtualEvent EventBuilder::ObjectClicked(const std::string& object_id, 
                                        glm::vec2 screen_pos, 
                                        glm::vec3 world_pos,
                                        int mouse_button) {
    auto event = CreateBaseEvent(VirtualEventType::ObjectClicked, object_id);
    event.screen_pos = screen_pos;
    event.world_pos = world_pos;
    event.mouse_button = mouse_button;
    return event;
}

VirtualEvent EventBuilder::ObjectDragged(const std::string& object_id,
                                        glm::vec2 screen_pos,
                                        glm::vec3 world_delta) {
    auto event = CreateBaseEvent(VirtualEventType::ObjectDragged, object_id);
    event.screen_pos = screen_pos;
    event.world_pos = world_delta; // Reuse world_pos for delta
    return event;
}

VirtualEvent EventBuilder::SelectionChanged(const std::vector<std::string>& selected_ids) {
    auto event = CreateBaseEvent(VirtualEventType::SelectionChanged);
    event.object_ids = selected_ids;
    event.object_id = selected_ids.empty() ? "" : selected_ids[0]; // Primary selection
    return event;
}

VirtualEvent EventBuilder::BackgroundClicked(glm::vec2 screen_pos, 
                                            glm::vec3 world_pos,
                                            int mouse_button) {
    auto event = CreateBaseEvent(VirtualEventType::BackgroundClicked);
    event.screen_pos = screen_pos;
    event.world_pos = world_pos;
    event.mouse_button = mouse_button;
    return event;
}

VirtualEvent EventBuilder::ObjectTransformed(const std::string& object_id,
                                            const glm::mat4& old_transform,
                                            const glm::mat4& new_transform) {
    auto event = CreateBaseEvent(VirtualEventType::ObjectTransformed, object_id);
    event.old_transform = old_transform;
    event.new_transform = new_transform;
    return event;
}

VirtualEvent EventBuilder::Custom(const std::string& object_id,
                                 const std::any& custom_data) {
    auto event = CreateBaseEvent(VirtualEventType::Custom, object_id);
    event.custom_data = custom_data;
    return event;
}

VirtualEvent EventBuilder::CreateBaseEvent(VirtualEventType type, const std::string& object_id) {
    VirtualEvent event;
    event.type = type;
    event.object_id = object_id;
    
    // Set timestamp
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    event.timestamp = std::chrono::duration<double>(duration).count();
    
    return event;
}

// ============================================================================
// EventSubscription Implementation
// ============================================================================

EventSubscription::EventSubscription(VirtualEventDispatcher* dispatcher, VirtualEventType type)
    : dispatcher_(dispatcher), type_(type), valid_(true) {
}

EventSubscription::~EventSubscription() {
    if (valid_ && dispatcher_) {
        dispatcher_->Unsubscribe(type_);
    }
}

EventSubscription::EventSubscription(EventSubscription&& other) noexcept
    : dispatcher_(other.dispatcher_), type_(other.type_), valid_(other.valid_) {
    other.valid_ = false;
}

EventSubscription& EventSubscription::operator=(EventSubscription&& other) noexcept {
    if (this != &other) {
        // Clean up current subscription
        if (valid_ && dispatcher_) {
            dispatcher_->Unsubscribe(type_);
        }
        
        // Move from other
        dispatcher_ = other.dispatcher_;
        type_ = other.type_;
        valid_ = other.valid_;
        other.valid_ = false;
    }
    return *this;
}

} // namespace quickviz