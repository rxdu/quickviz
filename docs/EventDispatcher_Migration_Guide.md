# EventDispatcher Migration Guide

This document describes the **BREAKING CHANGES** from the old EventDispatcher to the new unified EventDispatcher implementation.

## ⚠️ **BREAKING CHANGES NOTICE** ⚠️

**Legacy compatibility has been completely removed.** All existing code using the old EventDispatcher APIs will need to be updated to use the modern interface.

## Overview

The EventDispatcher has been completely redesigned with modern C++ features and a clean API. The old legacy compatibility layer has been removed to ensure a maintainable, high-performance codebase.

## Key Improvements

### New Features
- **Priority-based handler ordering**: Handlers are processed by priority (higher values first)
- **Event consumption mechanism**: Handlers can consume events to stop further propagation
- **Thread-safe operations**: All operations are protected by mutex
- **Type-safe event handling**: Template-based handlers for specific event types
- **Clean modern API**: Function-based handlers with lambda support
- **Enable/disable support**: Per-handler and global enable/disable functionality

### Architectural Changes
- **Header-only implementation**: No separate .cpp file needed
- **Modern InputDispatcher**: Clean interface specialized for InputEvents
- **No legacy compatibility**: All code must be updated to the new API

## Breaking Changes

### 1. EventDispatcher API Changes

**Old API (REMOVED):**
```cpp
// These no longer exist
using LegacyHandlerFunc = std::function<void(std::shared_ptr<BaseEvent>)>;
void RegisterHandler(const std::string& event_name, LegacyHandlerFunc handler);
void Dispatch(std::shared_ptr<BaseEvent> event) const;
static EventDispatcher& GetInstance(); // Singleton removed
```

**New API (REQUIRED):**
```cpp
using HandlerFunc = std::function<bool(std::shared_ptr<BaseEvent>)>;

// Register handlers with explicit names and priorities
void RegisterHandler(const std::string& event_name, HandlerFunc func, 
                    const std::string& handler_name, int priority = 0);

// Register handler objects  
void RegisterHandler(std::shared_ptr<Handler> handler);

// Template-based type-safe registration
template<typename EventType>
void RegisterTypedHandler(const std::string& handler_name,
                         typename TypedHandler<EventType>::TypedHandlerFunc func,
                         int priority = 0);

// Modern dispatch with consumption support
bool DispatchEvent(std::shared_ptr<BaseEvent> event);
```

### 2. InputDispatcher API Changes

**Old API (REMOVED):**
```cpp
// Legacy handler interfaces no longer supported
class EnhancedInputHandler { ... };
void RegisterHandler(std::shared_ptr<EnhancedInputHandler> handler);
std::shared_ptr<EnhancedInputHandler> GetHandler(const std::string& name);
void RegisterFunction(const std::string& name, std::function<bool(const InputEvent&)> func);
```

**New API (REQUIRED):**
```cpp
using InputHandler = std::function<bool(const InputEvent&)>;
using InputHandlerPtr = std::shared_ptr<EventDispatcher::Handler>;

// Register function handlers
InputHandlerPtr RegisterHandler(const std::string& name, InputHandler handler, int priority = 0);

// Specialized registration methods
InputHandlerPtr RegisterMouseHandler(const std::string& name, InputHandler handler, int priority = 0);
InputHandlerPtr RegisterKeyboardHandler(const std::string& name, InputHandler handler, int priority = 0);
InputHandlerPtr RegisterTypeHandler(const std::string& name, InputEventType event_type, 
                                   InputHandler handler, int priority = 0);

// Handler management
void UnregisterHandler(const std::string& name);
void UnregisterHandler(InputHandlerPtr handler);
InputHandlerPtr GetHandler(const std::string& name);
std::vector<InputHandlerPtr> GetHandlers();
```

### 3. Handler Function Signatures

**Old Signature (REMOVED):**
```cpp
void handler(std::shared_ptr<BaseEvent> event) { ... }
```

**New Signature (REQUIRED):**
```cpp
bool handler(std::shared_ptr<BaseEvent> event) { 
  // Process event
  return false; // false = don't consume, true = consume and stop propagation
}

// For InputDispatcher
bool inputHandler(const InputEvent& event) {
  // Process input event  
  return false; // false = don't consume, true = consume and stop propagation
}
```

## Migration Steps

### Step 1: Update EventDispatcher Usage

**Before:**
```cpp
// OLD CODE - NO LONGER WORKS
auto& dispatcher = EventDispatcher::GetInstance();
dispatcher.RegisterHandler("my_event", [](std::shared_ptr<BaseEvent> event) {
  // Handle event (void return)
});
dispatcher.Dispatch(event);
```

**After:**
```cpp
// NEW CODE - REQUIRED
EventDispatcher dispatcher;
dispatcher.RegisterHandler("my_event", 
  [](std::shared_ptr<BaseEvent> event) -> bool {
    // Handle event
    return false; // Don't consume
  }, "my_handler", 100); // Must provide name and priority

bool consumed = dispatcher.DispatchEvent(event);
```

### Step 2: Update InputDispatcher Usage

**Before:**
```cpp
// OLD CODE - NO LONGER WORKS
class MyInputHandler : public EnhancedInputHandler {
public:
  bool OnInputEvent(const InputEvent& event) override {
    // Handle input
    return false;
  }
  std::string GetName() const override { return "my_handler"; }
  int GetPriority() const override { return 10; }
};

auto handler = std::make_shared<MyInputHandler>();
input_dispatcher.RegisterHandler(handler);
```

**After:**
```cpp
// NEW CODE - REQUIRED
auto handler = input_dispatcher.RegisterHandler("my_handler",
  [](const InputEvent& event) -> bool {
    // Handle input
    return false; // Don't consume
  }, 10); // Priority
```

### Step 3: Update Mouse/Keyboard Handlers

**Before:**
```cpp
// OLD CODE - NO LONGER WORKS  
input_dispatcher.RegisterMouseHandler("mouse_handler", handler);
```

**After:**
```cpp
// NEW CODE - REQUIRED
auto mouse_handler = input_dispatcher.RegisterMouseHandler("mouse_handler",
  [](const InputEvent& event) -> bool {
    // This will only be called for mouse events
    return false;
  }, 10);
```

### Step 4: Update Event Consumption

**Before:**
```cpp
// OLD CODE - Event consumption was not supported
```

**After:**
```cpp
// NEW CODE - Modern consumption support
auto handler = dispatcher.RegisterHandler("handler",
  [](std::shared_ptr<BaseEvent> event) -> bool {
    // Process event
    if (should_consume) {
      return true; // Consume - stops further processing
    }
    return false; // Don't consume - allow other handlers
  }, "my_handler", 100);

bool consumed = dispatcher.DispatchEvent(event);
if (consumed) {
  // Event was consumed by a handler
}
```

## New Features Usage

### Priority-Based Processing

```cpp
// High priority handlers run first
dispatcher.RegisterHandler("critical", critical_handler, "critical_handler", 1000);
dispatcher.RegisterHandler("normal", normal_handler, "normal_handler", 100);
dispatcher.RegisterHandler("low", low_handler, "low_handler", 10);
```

### Type-Safe Handlers

```cpp
// Only called for InputEvent objects
dispatcher.RegisterTypedHandler<InputEvent>("input_handler",
  [](std::shared_ptr<InputEvent> event) -> bool {
    // Type-safe access to InputEvent methods
    return false;
  }, 50);
```

### Handler Management

```cpp
// Get handler reference
auto handler = dispatcher.GetHandler("my_handler");
if (handler) {
  handler->SetEnabled(false); // Temporarily disable
  handler->SetPriority(200);  // Change priority
}

// Remove handlers
dispatcher.UnregisterHandler("my_handler");
// or
dispatcher.UnregisterHandler(handler);
```

## Removed Features

The following features have been **completely removed**:

1. **Legacy handler function type** (`std::function<void(...)>`)
2. **Legacy dispatch method** (`Dispatch()` without consumption)
3. **Singleton access** (`GetInstance()`)
4. **EnhancedInputHandler interface** (replaced with function-based handlers)
5. **Automatic handler naming** (all handlers must have explicit names)
6. **Legacy compatibility adapter classes**

## Testing Your Migration

After updating your code, verify the migration with these checks:

1. **Compilation**: All handler functions must return `bool`
2. **Registration**: All handlers must have explicit names and priorities
3. **Dispatch**: Use `DispatchEvent()` and check return value for consumption
4. **Functionality**: Verify event handling works as expected
5. **Performance**: Modern API should show performance improvements

## Example: Complete Migration

**Before (Old API):**
```cpp
class MyEventSystem {
  EventDispatcher& dispatcher = EventDispatcher::GetInstance();
  
  void Setup() {
    dispatcher.RegisterHandler("mouse_click", 
      [this](std::shared_ptr<BaseEvent> event) {
        this->HandleMouseClick(event);
      });
  }
  
  void HandleMouseClick(std::shared_ptr<BaseEvent> event) {
    // Process event
  }
  
  void SendEvent() {
    dispatcher.Dispatch(mouse_event);
  }
};
```

**After (New API):**
```cpp
class MyEventSystem {
  EventDispatcher dispatcher; // No longer singleton
  
  void Setup() {
    dispatcher.RegisterHandler("mouse_click",
      [this](std::shared_ptr<BaseEvent> event) -> bool {
        return this->HandleMouseClick(event);
      }, "mouse_handler", 100); // Explicit name and priority
  }
  
  bool HandleMouseClick(std::shared_ptr<BaseEvent> event) {
    // Process event
    return false; // Don't consume by default
  }
  
  void SendEvent() {
    bool consumed = dispatcher.DispatchEvent(mouse_event);
    if (!consumed) {
      // Handle unconsumbed event if needed
    }
  }
};
```

## Support

This migration **requires code changes** in all projects using the old EventDispatcher API. The changes are necessary to:

- Improve performance and maintainability
- Provide modern C++ features
- Enable new functionality like event consumption and priorities
- Remove technical debt from legacy compatibility layers

For questions about specific migration scenarios, refer to:
- The comprehensive unit tests in `src/core/test/test_enhanced_event_system.cpp`
- The modern InputDispatcher examples in `src/core/test/unit_test/test_input_event.cpp`
- The QuickViz project documentation in `CLAUDE.md`