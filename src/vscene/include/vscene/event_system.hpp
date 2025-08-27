/*
 * @file event_system.hpp
 * @date August 27, 2025
 * @brief Event system for virtual scene interactions
 * 
 * The event system provides a clean way for applications to respond to
 * virtual scene interactions (clicks, selections, hover, etc.) without
 * tight coupling to the rendering or UI systems.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_EVENT_SYSTEM_HPP
#define QUICKVIZ_EVENT_SYSTEM_HPP

#include <functional>
#include <vector>
#include <unordered_map>
#include <any>
#include <string>
#include <glm/glm.hpp>

namespace quickviz {

/**
 * @brief Types of virtual scene events
 */
enum class VirtualEventType {
    // Object interaction events
    ObjectClicked,
    ObjectDoubleClicked,
    ObjectRightClicked,
    ObjectDragged,
    ObjectHoverEnter,
    ObjectHoverExit,
    
    // Selection events
    SelectionChanged,
    SelectionCleared,
    
    // Background events
    BackgroundClicked,
    BackgroundRightClicked,
    BackgroundDoubleClicked,
    
    // Transform events
    ObjectTransformed,
    ObjectMoved,
    ObjectRotated,
    ObjectScaled,
    
    // Scene events
    SceneChanged,
    ObjectAdded,
    ObjectRemoved,
    
    // Custom events (for application-specific needs)
    Custom
};

/**
 * @brief Virtual scene event data
 * 
 * VirtualEvent contains all information about an interaction event,
 * allowing applications to respond appropriately to user actions.
 */
struct VirtualEvent {
    VirtualEventType type;
    
    // Object information
    std::string object_id;              // Empty for background events
    std::vector<std::string> object_ids; // For multi-selection events
    
    // Spatial information
    glm::vec2 screen_pos;               // Mouse position in screen coordinates
    glm::vec3 world_pos;                // 3D world position (if applicable)
    glm::vec3 world_normal;             // Surface normal at hit point (if applicable)
    
    // Input state
    int mouse_button = 0;               // Which mouse button (0=left, 1=right, 2=middle)
    bool ctrl_pressed = false;
    bool shift_pressed = false;
    bool alt_pressed = false;
    
    // Event-specific data
    std::any custom_data;               // For custom events or additional data
    
    // Transform information (for transform events)
    glm::mat4 old_transform;
    glm::mat4 new_transform;
    
    // Timing
    double timestamp = 0.0;             // Event timestamp (seconds since start)
};

/**
 * @brief Event dispatcher for virtual scene events
 * 
 * EventDispatcher manages event subscriptions and dispatching for the
 * virtual scene system. Applications subscribe to events they care about
 * and receive callbacks when those events occur.
 */
class EventDispatcher {
public:
    using EventHandler = std::function<void(const VirtualEvent&)>;
    using EventFilter = std::function<bool(const VirtualEvent&)>;

public:
    // Subscription management
    void Subscribe(VirtualEventType type, EventHandler handler);
    void Subscribe(VirtualEventType type, EventHandler handler, EventFilter filter);
    void Unsubscribe(VirtualEventType type);
    void UnsubscribeAll();
    
    // Event dispatching
    void Dispatch(const VirtualEvent& event);
    void DispatchAsync(const VirtualEvent& event); // For thread-safe async dispatch
    
    // Bulk operations
    void BeginBatch();  // Start batching events
    void EndBatch();    // Dispatch all batched events
    void ClearBatch();  // Clear batched events without dispatching
    
    // Statistics
    size_t GetSubscriberCount(VirtualEventType type) const;
    size_t GetTotalSubscribers() const;
    size_t GetEventsDispatchedCount() const { return events_dispatched_; }

private:
    using HandlerList = std::vector<std::pair<EventHandler, EventFilter>>;
    std::unordered_map<VirtualEventType, HandlerList> subscribers_;
    
    // Batching support
    std::vector<VirtualEvent> batched_events_;
    bool batching_enabled_ = false;
    
    // Statistics
    size_t events_dispatched_ = 0;
    
    // Internal helpers
    void DispatchToSubscribers(const VirtualEvent& event);
    bool PassesFilter(const VirtualEvent& event, const EventFilter& filter) const;
};

/**
 * @brief Convenience event builder for common event types
 */
class EventBuilder {
public:
    static VirtualEvent ObjectClicked(const std::string& object_id, 
                                    glm::vec2 screen_pos, 
                                    glm::vec3 world_pos,
                                    int mouse_button = 0);
    
    static VirtualEvent ObjectDragged(const std::string& object_id,
                                    glm::vec2 screen_pos,
                                    glm::vec3 world_delta);
    
    static VirtualEvent SelectionChanged(const std::vector<std::string>& selected_ids);
    
    static VirtualEvent BackgroundClicked(glm::vec2 screen_pos, 
                                        glm::vec3 world_pos,
                                        int mouse_button = 0);
    
    static VirtualEvent ObjectTransformed(const std::string& object_id,
                                        const glm::mat4& old_transform,
                                        const glm::mat4& new_transform);
    
    static VirtualEvent Custom(const std::string& object_id,
                             const std::any& custom_data);

private:
    static VirtualEvent CreateBaseEvent(VirtualEventType type, const std::string& object_id = "");
};

/**
 * @brief Event subscription helper with RAII semantics
 * 
 * EventSubscription automatically unsubscribes when destroyed,
 * making it safe to use in automatic scope.
 */
class EventSubscription {
public:
    EventSubscription(EventDispatcher* dispatcher, VirtualEventType type);
    ~EventSubscription();
    
    // No copying (move-only)
    EventSubscription(const EventSubscription&) = delete;
    EventSubscription& operator=(const EventSubscription&) = delete;
    
    // Move semantics
    EventSubscription(EventSubscription&& other) noexcept;
    EventSubscription& operator=(EventSubscription&& other) noexcept;

private:
    EventDispatcher* dispatcher_;
    VirtualEventType type_;
    bool valid_;
};

} // namespace quickviz

#endif // QUICKVIZ_EVENT_SYSTEM_HPP