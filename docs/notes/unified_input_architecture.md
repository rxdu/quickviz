# Unified Input Architecture for QuickViz

## Problem Statement

Currently, input handling is inconsistent across different input types:
- **Mouse/Keyboard**: New ImGui-centric `InputEvent` system
- **Joystick**: Legacy callback-based `JoystickInput` system

## Proposed Solution: Unified InputEvent System

### 1. Extend InputEvent for All Input Types

```cpp
enum class InputEventType {
  // Mouse events
  kMousePress,
  kMouseRelease,
  kMouseMove,
  kMouseDrag,
  kMouseWheel,
  
  // Keyboard events
  kKeyPress,
  kKeyRelease,
  
  // Joystick events (NEW)
  kJoystickConnected,
  kJoystickDisconnected,
  kJoystickAxisMove,
  kJoystickButtonPress,
  kJoystickButtonRelease,
  kJoystickHatMove
};

class InputEvent {
  // ... existing mouse/keyboard fields ...
  
  // New joystick-specific fields
  int joystick_id_ = -1;
  std::string joystick_name_;
  std::vector<float> axis_values_;
  std::vector<bool> button_states_;
  int hat_state_ = 0;
  
  // Getters for joystick data
  bool IsJoystickEvent() const;
  int GetJoystickId() const;
  const std::string& GetJoystickName() const;
  const std::vector<float>& GetAxisValues() const;
  float GetAxisValue(int axis) const;
  const std::vector<bool>& GetButtonStates() const;
  bool GetButtonState(int button) const;
  int GetHatState() const;
};
```

### 2. Unified Input Polling

```cpp
class ImGuiInputUtils {
  // ... existing mouse/keyboard polling ...
  
  // New joystick polling methods
  static void PollJoystickEvents(std::vector<InputEvent>& events);
  static void PollJoystickConnections(std::vector<InputEvent>& events);
  
  // Updated unified polling
  static void PollAllEvents(std::vector<InputEvent>& events) {
    PollMouseEvents(events);
    PollKeyboardEvents(events);
    PollJoystickEvents(events);
  }
};
```

### 3. Consistent Handler Interface

```cpp
// Single unified handler interface
class InputEventHandler {
 public:
  virtual bool OnInputEvent(const InputEvent& event) = 0;
  
  // Optional convenience methods for derived classes
  virtual bool OnMouseEvent(const InputEvent& event) { return false; }
  virtual bool OnKeyboardEvent(const InputEvent& event) { return false; }
  virtual bool OnJoystickEvent(const InputEvent& event) { return false; }
};

// Example implementation
class GamepadHandler : public InputEventHandler {
 public:
  bool OnInputEvent(const InputEvent& event) override {
    if (event.IsJoystickEvent()) {
      return OnJoystickEvent(event);
    }
    return false;
  }
  
 protected:
  bool OnJoystickEvent(const InputEvent& event) override {
    switch (event.GetType()) {
      case InputEventType::kJoystickButtonPress:
        return HandleButtonPress(event.GetButtonOrKey());
      case InputEventType::kJoystickAxisMove:
        return HandleAxisMove(event.GetJoystickId(), 
                             event.GetAxisValues());
      default:
        return false;
    }
  }
};
```

### 4. Unified Processing Flow

All input types follow the same pattern:
```
Input Source → ImGuiInputUtils → InputEvent → InputDispatcher → InputEventHandler
```

**For Mouse/Keyboard**: `ImGui polling` → `InputEvent`
**For Joystick**: `GLFW polling` → `InputEvent` 

### 5. Migration Strategy

1. **Phase 1**: Extend `InputEvent` with joystick fields
2. **Phase 2**: Add joystick polling to `ImGuiInputUtils`  
3. **Phase 3**: Create adapter for legacy `InputHandler::OnJoystickUpdate`
4. **Phase 4**: Migrate existing joystick handlers to unified system
5. **Phase 5**: Deprecate legacy joystick interface

## Benefits

✅ **Consistent API** across all input types
✅ **Single processing pipeline** with unified priorities
✅ **Action mapping** works for all input types (mouse, keyboard, joystick)
✅ **Event consumption** works consistently
✅ **ImGui integration** for all input (respects capture flags)
✅ **Backward compatibility** through adapters

## Implementation Notes

- **Polling vs Callbacks**: Move to polling-based approach for consistency
- **Performance**: Cache joystick state, only generate events on changes
- **ImGui Integration**: Check `io.ConfigFlags & ImGuiConfigFlags_NavEnableGamepad`
- **Action Mapping**: Extend `InputMapping` to support joystick actions