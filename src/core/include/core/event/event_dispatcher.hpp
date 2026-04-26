/*
 * @file event_dispatcher.hpp
 * @date 10/7/24
 * @brief Enhanced event dispatcher with priority, consumption, and thread safety
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_EVENT_DISPATCHER_HPP
#define QUICKVIZ_EVENT_DISPATCHER_HPP

#include <string>
#include <memory>
#include <vector>
#include <functional>
#include <unordered_map>
#include <algorithm>
#include <mutex>
#include <typeindex>

#include "core/event/event.hpp"

namespace quickviz {

// Forward declarations
class BaseEvent;

/**
 * @brief Enhanced event dispatcher with priority, consumption, and thread safety
 * 
 * Features:
 * - Priority-based handler ordering
 * - Event consumption mechanism
 * - Thread-safe operations
 * - Both function and interface-based handlers
 * - Type-safe event filtering
 * - Enable/disable per handler and globally
 */
class EventDispatcher {
 public:
  // Handler function type (returns bool for consumption support)
  using HandlerFunc = std::function<bool(std::shared_ptr<BaseEvent>)>;
  
  
  /**
   * @brief Base class for event handlers with priority support
   */
  class Handler {
   public:
    Handler(const std::string& name, int priority = 0) 
        : name_(name), priority_(priority), enabled_(true) {}
    virtual ~Handler() = default;
    
    // Core interface
    virtual bool HandleEvent(std::shared_ptr<BaseEvent> event) = 0;
    virtual bool CanHandle(std::shared_ptr<BaseEvent> event) const = 0;
    
    // Properties
    const std::string& GetName() const { return name_; }
    int GetPriority() const { return priority_; }
    void SetPriority(int priority) { priority_ = priority; }
    
    bool IsEnabled() const { return enabled_; }
    void SetEnabled(bool enabled) { enabled_ = enabled; }
    
   protected:
    std::string name_;
    int priority_;
    bool enabled_;
  };
  
  /**
   * @brief Function-based handler wrapper
   */
  class FunctionHandler : public Handler {
   public:
    FunctionHandler(const std::string& name, HandlerFunc func, 
                   const std::string& event_name = "", int priority = 0)
        : Handler(name, priority), func_(func), event_name_(event_name) {}
    
    bool HandleEvent(std::shared_ptr<BaseEvent> event) override {
      return func_(event);
    }
    
    bool CanHandle(std::shared_ptr<BaseEvent> event) const override {
      // If event_name is specified, filter by name
      if (!event_name_.empty()) {
        return event->GetName() == event_name_;
      }
      return true;  // Handle all events if no filter specified
    }
    
   private:
    HandlerFunc func_;
    std::string event_name_;
  };
  
  
  /**
   * @brief Type-safe handler for specific event types
   */
  template<typename EventType>
  class TypedHandler : public Handler {
   public:
    using TypedHandlerFunc = std::function<bool(std::shared_ptr<EventType>)>;
    
    TypedHandler(const std::string& name, TypedHandlerFunc func, int priority = 0)
        : Handler(name, priority), func_(func) {}
    
    bool HandleEvent(std::shared_ptr<BaseEvent> event) override {
      auto typed_event = std::dynamic_pointer_cast<EventType>(event);
      if (typed_event) {
        return func_(typed_event);
      }
      return false;
    }
    
    bool CanHandle(std::shared_ptr<BaseEvent> event) const override {
      return std::dynamic_pointer_cast<EventType>(event) != nullptr;
    }
    
   private:
    TypedHandlerFunc func_;
  };
  
 public:
  EventDispatcher() = default;
  ~EventDispatcher() = default;

  // === Handler Registration ===
  
  /**
   * @brief Register a custom handler object
   */
  void RegisterHandler(std::shared_ptr<Handler> handler) {
    std::lock_guard<std::mutex> lock(mutex_);
    handlers_.push_back(handler);
    SortHandlers();
  }
  
  /**
   * @brief Register a function handler with consumption support
   * @param event_name Optional event name filter (empty = all events)
   * @param func Handler function returning bool (true = consume event)
   * @param handler_name Unique name for the handler
   * @param priority Handler priority (higher = processed first)
   */
  void RegisterHandler(const std::string& event_name, HandlerFunc func, 
                      const std::string& handler_name, int priority = 0) {
    auto handler = std::make_shared<FunctionHandler>(handler_name, func, event_name, priority);
    RegisterHandler(handler);
  }
  
  
  /**
   * @brief Register a type-safe handler for specific event types
   */
  template<typename EventType>
  void RegisterTypedHandler(const std::string& handler_name,
                           typename TypedHandler<EventType>::TypedHandlerFunc func,
                           int priority = 0) {
    auto handler = std::make_shared<TypedHandler<EventType>>(handler_name, func, priority);
    RegisterHandler(handler);
  }
  
  // === Handler Management ===
  
  void UnregisterHandler(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);
    handlers_.erase(
        std::remove_if(handlers_.begin(), handlers_.end(),
                       [&name](const std::shared_ptr<Handler>& handler) {
                         return handler->GetName() == name;
                       }),
        handlers_.end());
  }
  
  void UnregisterHandler(std::shared_ptr<Handler> handler) {
    std::lock_guard<std::mutex> lock(mutex_);
    handlers_.erase(std::remove(handlers_.begin(), handlers_.end(), handler),
                    handlers_.end());
  }
  
  void ClearHandlers() {
    std::lock_guard<std::mutex> lock(mutex_);
    handlers_.clear();
  }
  
  std::shared_ptr<Handler> GetHandler(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = std::find_if(
        handlers_.begin(), handlers_.end(),
        [&name](const std::shared_ptr<Handler>& handler) {
          return handler->GetName() == name;
        });
    return (it != handlers_.end()) ? *it : nullptr;
  }
  
  std::vector<std::shared_ptr<Handler>> GetHandlers() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return handlers_;
  }
  
  // === Event Dispatching ===
  
  /**
   * @brief Dispatch event to all relevant handlers in priority order
   * @param event Event to dispatch
   * @return true if event was consumed by a handler
   */
  bool DispatchEvent(std::shared_ptr<BaseEvent> event) {
    if (!enabled_ || !event) return false;
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Process handlers in priority order
    for (auto& handler : handlers_) {
      if (!handler->IsEnabled()) continue;
      if (!handler->CanHandle(event)) continue;
      
      // Let handler process the event
      bool consumed = handler->HandleEvent(event);
      
      // If handler consumed the event, stop propagation
      if (consumed) {
        return true;
      }
    }
    
    return false;
  }
  
  
  // === Global Control ===
  
  void SetEnabled(bool enabled) { enabled_ = enabled; }
  bool IsEnabled() const { return enabled_; }
  
  void SetHandlerPriority(const std::string& name, int priority) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto handler = GetHandler(name);
    if (handler) {
      handler->SetPriority(priority);
      SortHandlers();
    }
  }
  
  // === Statistics ===
  
  size_t GetHandlerCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return handlers_.size();
  }
  
  size_t GetEnabledHandlerCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::count_if(handlers_.begin(), handlers_.end(),
                        [](const std::shared_ptr<Handler>& h) {
                          return h->IsEnabled();
                        });
  }
  
  
 private:
  void SortHandlers() {
    // Sort by priority (higher priority first)
    std::sort(handlers_.begin(), handlers_.end(),
              [](const std::shared_ptr<Handler>& a,
                 const std::shared_ptr<Handler>& b) {
                return a->GetPriority() > b->GetPriority();
              });
  }
  
  std::vector<std::shared_ptr<Handler>> handlers_;
  bool enabled_ = true;
  mutable std::mutex mutex_;
};

}  // namespace quickviz

#endif  // QUICKVIZ_EVENT_DISPATCHER_HPP