/**
 * @file interaction_tool.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-09-02
 * @brief Base interface for interactive 3D scene tools
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_INTERACTION_TOOL_HPP
#define QUICKVIZ_INTERACTION_TOOL_HPP

#include <string>
#include <functional>

#include <glm/glm.hpp>
#include "imview/input/input_dispatcher.hpp"
#include "core/event/input_event.hpp"

namespace quickviz {

// Forward declarations
class SceneManager;

/**
 * @brief Base interface for interactive 3D scene tools
 * 
 * Interactive tools provide high-level user interaction patterns that build
 * on top of the low-level selection and input systems. They manage tool state,
 * visual feedback, and user workflow patterns.
 * 
 * Design Principles:
 * - Tools are stateful: Active/Inactive/Hover states
 * - Tools provide visual feedback during interaction
 * - Tools integrate with priority-based input handling
 * - Tools can be switched/activated through UI
 */
class InteractionTool : public InputEventHandler {
public:
    /**
     * @brief Tool activation state
     */
    enum class State {
        kInactive,     // Tool is available but not active
        kActive,       // Tool is active and handling input
        kHover,        // Tool is providing hover feedback
        kWorking       // Tool is in middle of multi-step operation
    };

    /**
     * @brief Tool cursor types for visual feedback
     */
    enum class CursorType {
        kDefault,      // System default cursor
        kCrosshair,    // Precision selection cursor
        kHand,         // Grabbable item cursor
        kMove,         // Movement/drag cursor
        kResize,       // Resize cursor
        kWorking       // Processing/working cursor
    };

    explicit InteractionTool(const std::string& name, SceneManager* scene_manager);
    virtual ~InteractionTool() = default;

    // InputEventHandler interface (priority-based event handling)
    bool OnInputEvent(const InputEvent& event) override;
    std::string GetName() const override { return name_; }
    bool IsEnabled() const override { return enabled_; }
    int GetPriority() const override { 
        // Active tools get higher priority than inactive ones
        return (state_ == State::kActive || state_ == State::kWorking) ? priority_ + 50 : priority_; 
    }

    // === Tool Lifecycle ===
    
    /**
     * @brief Activate the tool (called when user selects this tool)
     */
    virtual void OnActivate();
    
    /**
     * @brief Deactivate the tool (called when user selects different tool)
     */
    virtual void OnDeactivate();
    
    /**
     * @brief Render tool-specific visual feedback
     * @param projection Projection matrix for 3D rendering
     * @param view View matrix for 3D rendering
     * @note Called during scene rendering when tool is active
     */
    virtual void OnRender(const glm::mat4& projection, const glm::mat4& view) {}

    // === Tool State Management ===
    
    /**
     * @brief Get current tool state
     */
    State GetState() const { return state_; }
    
    /**
     * @brief Enable/disable tool (disabled tools don't respond to input)
     */
    void SetEnabled(bool enabled) { enabled_ = enabled; }
    
    /**
     * @brief Get tool display name for UI
     */
    const std::string& GetDisplayName() const { return display_name_; }
    
    /**
     * @brief Set tool display name for UI
     */
    void SetDisplayName(const std::string& name) { display_name_ = name; }
    
    /**
     * @brief Get tool description/tooltip
     */
    const std::string& GetDescription() const { return description_; }
    
    /**
     * @brief Set tool description/tooltip
     */
    void SetDescription(const std::string& description) { description_ = description; }

    // === Visual Feedback ===
    
    /**
     * @brief Get current cursor type for this tool
     */
    virtual CursorType GetCursorType() const { return CursorType::kDefault; }
    
    /**
     * @brief Check if tool should show hover feedback
     */
    virtual bool ShowsHoverFeedback() const { return true; }

protected:
    // === Event Handling (override in derived classes) ===
    
    /**
     * @brief Handle mouse events (button press/release, movement)
     * @param event Mouse event
     * @return true to consume event, false to pass through
     */
    virtual bool OnMouseEvent(const InputEvent& event) { return false; }
    
    /**
     * @brief Handle keyboard events
     * @param event Keyboard event
     * @return true to consume event, false to pass through
     */
    virtual bool OnKeyboardEvent(const InputEvent& event) { return false; }
    
    /**
     * @brief Handle tool activation (override for tool-specific setup)
     */
    virtual void DoActivate() {}
    
    /**
     * @brief Handle tool deactivation (override for tool-specific cleanup)
     */
    virtual void DoDeactivate() {}

    // === Utility Methods ===
    
    /**
     * @brief Convert screen coordinates to normalized device coordinates
     * @param screen_pos Screen position (window coordinates)
     * @param viewport_size Viewport size
     * @return NDC coordinates (-1 to +1)
     */
    glm::vec2 ScreenToNDC(const glm::vec2& screen_pos, const glm::vec2& viewport_size) const;
    
    /**
     * @brief Get mouse position from input event
     * @param event Input event (must be mouse event)
     * @return Screen coordinates, or (0,0) if not a mouse event
     */
    glm::vec2 GetMousePosition(const InputEvent& event) const;
    
    /**
     * @brief Check if modifier keys are pressed
     * @param event Input event
     * @param modifiers Modifier flags to check
     * @return true if all specified modifiers are pressed
     */
    bool HasModifiers(const InputEvent& event, ModifierKeys modifiers) const;

    // === State Management ===
    
    /**
     * @brief Change tool state with validation
     * @param new_state New state to transition to
     */
    void SetState(State new_state);

    // === Protected Members ===
    
    std::string name_;                // Unique tool identifier
    std::string display_name_;        // User-visible name
    std::string description_;         // Tool description/tooltip
    SceneManager* scene_manager_;     // Scene manager reference
    State state_ = State::kInactive;  // Current tool state
    bool enabled_ = true;             // Tool enabled state
    int priority_ = 100;              // Input event priority (default: medium)
};

/**
 * @brief Tool state change callback
 */
using ToolStateCallback = std::function<void(InteractionTool* tool, InteractionTool::State old_state, InteractionTool::State new_state)>;

/**
 * @brief Manager for interactive tools in a 3D scene
 * 
 * Manages tool lifecycle, activation/deactivation, and provides
 * a centralized way to switch between different interaction modes.
 */
class ToolManager {
public:
    explicit ToolManager(SceneManager* scene_manager);
    ~ToolManager() = default;

    /**
     * @brief Register a tool with the manager
     * @param tool Shared pointer to tool (manager will hold reference)
     */
    void RegisterTool(std::shared_ptr<InteractionTool> tool);
    
    /**
     * @brief Unregister tool by name
     * @param name Tool name to remove
     */
    void UnregisterTool(const std::string& name);
    
    /**
     * @brief Get tool by name
     * @param name Tool name
     * @return Tool pointer, or nullptr if not found
     */
    std::shared_ptr<InteractionTool> GetTool(const std::string& name);
    
    /**
     * @brief Activate a tool (deactivates current active tool)
     * @param name Tool name to activate
     * @return true if tool was found and activated
     */
    bool ActivateTool(const std::string& name);
    
    /**
     * @brief Deactivate current active tool
     */
    void DeactivateCurrentTool();
    
    /**
     * @brief Get currently active tool
     * @return Active tool, or nullptr if none active
     */
    std::shared_ptr<InteractionTool> GetActiveTool() const { return active_tool_; }
    
    /**
     * @brief Get all registered tools
     */
    std::vector<std::shared_ptr<InteractionTool>> GetAllTools() const;
    
    /**
     * @brief Set callback for tool state changes
     * @param callback Function to call when tool state changes
     */
    void SetStateChangeCallback(ToolStateCallback callback) { state_callback_ = callback; }
    
    /**
     * @brief Render active tool's visual feedback
     * @param projection Projection matrix
     * @param view View matrix
     */
    void RenderActiveTool(const glm::mat4& projection, const glm::mat4& view);

private:
    SceneManager* scene_manager_;
    std::vector<std::shared_ptr<InteractionTool>> tools_;
    std::shared_ptr<InteractionTool> active_tool_;
    ToolStateCallback state_callback_;
    
    void OnToolStateChanged(InteractionTool* tool, InteractionTool::State old_state, InteractionTool::State new_state);
};

} // namespace quickviz

#endif // QUICKVIZ_INTERACTION_TOOL_HPP