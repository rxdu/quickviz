/*
 * @file input_dispatcher.hpp
 * @date 9/1/25
 * @brief Central input event dispatcher for imview module
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef IMVIEW_INPUT_DISPATCHER_HPP
#define IMVIEW_INPUT_DISPATCHER_HPP

#include <memory>
#include <vector>
#include <string>
#include <functional>
#include <algorithm>

#include "core/event/input_event.hpp"
#include "imview/interface/input_handler.hpp"

namespace quickviz {

/**
 * @brief Priority-based input event handler interface for imview
 * 
 * Extends the basic InputHandler interface with priority support
 * for proper event ordering and consumption.
 */
class InputEventHandler {
 public:
  virtual ~InputEventHandler() = default;
  
  /**
   * @brief Get handler priority (higher = processed first)
   */
  virtual int GetPriority() const = 0;
  
  /**
   * @brief Handle input event
   * @param event The input event to handle
   * @return true to consume event (stop propagation), false to continue
   */
  virtual bool OnInputEvent(const InputEvent& event) = 0;
  
  /**
   * @brief Get handler name for identification
   */
  virtual std::string GetName() const = 0;
  
  /**
   * @brief Check if handler is enabled
   */
  virtual bool IsEnabled() const { return true; }
};

/**
 * @brief Central input event dispatcher for the imview module
 * 
 * Manages priority-based event handling with proper consumption semantics.
 * Handlers are processed in priority order (highest first) until one
 * consumes the event.
 */
class InputDispatcher {
 public:
  InputDispatcher() = default;
  ~InputDispatcher() = default;

  // Non-copyable
  InputDispatcher(const InputDispatcher&) = delete;
  InputDispatcher& operator=(const InputDispatcher&) = delete;

  /**
   * @brief Register an input event handler
   * @param handler Shared pointer to handler (manages lifetime)
   */
  void RegisterHandler(std::shared_ptr<InputEventHandler> handler);

  /**
   * @brief Unregister handler by name
   * @param name Handler name to remove
   */
  void UnregisterHandler(const std::string& name);

  /**
   * @brief Remove all handlers
   */
  void ClearHandlers();

  /**
   * @brief Dispatch event to registered handlers
   * @param event Event to dispatch
   * @return true if event was consumed by any handler
   */
  bool DispatchEvent(const InputEvent& event);

  /**
   * @brief Enable/disable entire dispatcher
   * @param enabled Whether dispatcher should process events
   */
  void SetEnabled(bool enabled) { enabled_ = enabled; }
  bool IsEnabled() const { return enabled_; }

  /**
   * @brief Get number of registered handlers
   */
  size_t GetHandlerCount() const { return handlers_.size(); }

  /**
   * @brief Get list of handler names (for debugging)
   */
  std::vector<std::string> GetHandlerNames() const;

 private:
  void SortHandlers();

  std::vector<std::shared_ptr<InputEventHandler>> handlers_;
  bool enabled_ = true;
  bool needs_sort_ = false;
};

/**
 * @brief Adapter to bridge legacy InputHandler to new InputEventHandler
 * 
 * This allows existing code using the old InputHandler interface
 * to work with the new event-driven system.
 */
class InputHandlerAdapter : public InputEventHandler {
 public:
  InputHandlerAdapter(InputHandler* legacy_handler, 
                      const std::string& name,
                      int priority = 0);

  // InputEventHandler interface
  int GetPriority() const override { return priority_; }
  bool OnInputEvent(const InputEvent& event) override;
  std::string GetName() const override { return name_; }
  bool IsEnabled() const override { return legacy_handler_ != nullptr; }

 private:
  InputHandler* legacy_handler_;
  std::string name_;
  int priority_;
};

}  // namespace quickviz

#endif  // IMVIEW_INPUT_DISPATCHER_HPP