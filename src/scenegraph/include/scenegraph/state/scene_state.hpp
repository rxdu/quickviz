/**
 * @file scene_state.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-09-03
 * @brief Core scene state management with modal operation modes
 *
 * Provides unified scene state management supporting both real-time
 * visualization and interactive editing through modal operation modes.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_SCENE_STATE_HPP
#define QUICKVIZ_SCENE_STATE_HPP

#include <memory>
#include <unordered_map>
#include <functional>
#include <mutex>
#include <glm/glm.hpp>

#include "scenegraph/command/command_stack.hpp"
#include "gldraw/interface/opengl_object.hpp"

namespace quickviz {

/**
 * @brief Unique identifier for scene objects
 */
using ObjectId = uint32_t;
constexpr ObjectId kInvalidObjectId = 0;

/**
 * @brief Operation mode for scene state management
 */
enum class OperationMode {
    /**
     * Direct mode - Zero overhead, immediate operations
     * - All operations bypass state tracking
     * - No undo/redo support
     * - Optimal for real-time visualization
     */
    kDirect,
    
    /**
     * Immediate mode - State tracked, no command history
     * - Operations tracked for synchronization
     * - No undo/redo support (commands not stored)
     * - Good for live updates with state consistency
     */
    kImmediate,
    
    /**
     * Recorded mode - Full state management with undo/redo
     * - All operations become undoable commands
     * - Complete history tracking
     * - Full interactive editing capabilities
     */
    kRecorded
};

/**
 * @brief Change notification callback
 */
using ChangeCallback = std::function<void(ObjectId, const std::string&)>;

/**
 * @brief Core scene state manager with modal operation modes
 * 
 * Provides unified interface for scene manipulation across different
 * operation modes, enabling both high-performance real-time updates
 * and full-featured interactive editing.
 * 
 * Key Features:
 * - Modal operation (Direct/Immediate/Recorded)
 * - Zero overhead in Direct mode for real-time use cases
 * - Full undo/redo support in Recorded mode
 * - Consistent API across all modes
 * - Thread-safe object registration and lookups
 * 
 * Usage Examples:
 * ```cpp
 * // Real-time robotics visualization (zero overhead)
 * scene.SetMode(OperationMode::kDirect);
 * scene.SetTransform(robot_id, current_pose);  // No command overhead
 * 
 * // Interactive editing with undo support
 * scene.SetMode(OperationMode::kRecorded);
 * scene.SetTransform(object_id, new_transform);  // Creates undoable command
 * scene.Undo();  // Revert transformation
 * ```
 */
class SceneState {
public:
    /**
     * @brief Configuration for scene state behavior
     */
    struct Config {
        OperationMode mode = OperationMode::kDirect;
        
        // Command stack configuration (only used in Recorded mode)
        size_t max_commands = 1000;
        size_t memory_limit_bytes = 100 * 1024 * 1024;  // 100MB
        bool auto_compress = true;
        bool track_persistent_only = true;
        
        // Change notification configuration
        bool enable_change_notifications = false;
    };
    
    /**
     * @brief Construct scene state with default configuration
     */
    SceneState();
    
    /**
     * @brief Construct scene state with custom configuration
     * @param config Configuration settings
     */
    explicit SceneState(const Config& config);
    
    /**
     * @brief Destructor - ensures proper cleanup
     */
    ~SceneState();
    
    // === Mode Management ===
    
    /**
     * @brief Set operation mode
     * @param mode New operation mode
     * @note Mode changes are immediate and affect all subsequent operations
     */
    void SetMode(OperationMode mode);
    
    /**
     * @brief Get current operation mode
     * @return Current operation mode
     */
    OperationMode GetMode() const { return config_.mode; }
    
    /**
     * @brief Check if mode supports undo/redo
     * @return true if current mode supports undo/redo
     */
    bool SupportsUndo() const { return config_.mode == OperationMode::kRecorded; }
    
    // === Object Registration ===
    
    /**
     * @brief Register an OpenGL object in the scene
     * @param object Shared pointer to object (ownership shared)
     * @return Unique object identifier
     * @note Registration is immediate in all modes
     */
    ObjectId RegisterObject(std::shared_ptr<OpenGlObject> object);
    
    /**
     * @brief Unregister an object from the scene
     * @param id Object identifier to remove
     * @return true if object was found and removed
     * @note Unregistration respects current operation mode
     */
    bool UnregisterObject(ObjectId id);
    
    /**
     * @brief Get object by ID
     * @param id Object identifier
     * @return Shared pointer to object, or nullptr if not found
     * @note Lookup is thread-safe and immediate in all modes
     */
    std::shared_ptr<OpenGlObject> GetObject(ObjectId id) const;
    
    /**
     * @brief Check if object exists in scene
     * @param id Object identifier
     * @return true if object is registered
     */
    bool HasObject(ObjectId id) const;
    
    /**
     * @brief Get count of registered objects
     * @return Number of registered objects
     */
    size_t GetObjectCount() const;
    
    // === State Manipulation ===
    
    /**
     * @brief Set transform for an object
     * @param id Object identifier
     * @param transform New transformation matrix
     * @return true if operation succeeded
     * 
     * Behavior by mode:
     * - Direct: Immediate update, no tracking
     * - Immediate: Update with change notification
     * - Recorded: Create undoable TransformCommand
     */
    bool SetTransform(ObjectId id, const glm::mat4& transform);
    
    /**
     * @brief Get transform for an object
     * @param id Object identifier
     * @return Current transformation matrix, or identity if object not found
     */
    glm::mat4 GetTransform(ObjectId id) const;
    
    /**
     * @brief Set visibility for an object
     * @param id Object identifier
     * @param visible Visibility state
     * @return true if operation succeeded
     */
    bool SetVisible(ObjectId id, bool visible);
    
    /**
     * @brief Get visibility for an object
     * @param id Object identifier
     * @return Visibility state, false if object not found
     */
    bool IsVisible(ObjectId id) const;
    
    // === Command Operations (Recorded mode only) ===
    
    /**
     * @brief Execute custom command
     * @param command Command to execute
     * @return true if command was executed (only in Recorded mode)
     */
    bool ExecuteCommand(std::unique_ptr<Command> command);
    
    /**
     * @brief Undo last operation
     * @return true if undo succeeded
     */
    bool Undo();
    
    /**
     * @brief Redo next operation
     * @return true if redo succeeded  
     */
    bool Redo();
    
    /**
     * @brief Check if undo is available
     * @return true if undo is possible
     */
    bool CanUndo() const;
    
    /**
     * @brief Check if redo is available
     * @return true if redo is possible
     */
    bool CanRedo() const;
    
    /**
     * @brief Get description of next undo operation
     * @return Description string, empty if no undo available
     */
    std::string GetUndoDescription() const;
    
    /**
     * @brief Get description of next redo operation
     * @return Description string, empty if no redo available
     */
    std::string GetRedoDescription() const;
    
    /**
     * @brief Get command stack statistics
     * @return Statistics object (only meaningful in Recorded mode)
     */
    CommandStack::Statistics GetCommandStatistics() const;
    
    // === Compound Operations ===
    
    /**
     * @brief Begin compound operation
     * @param description Description for the compound operation
     * @return true if compound operation started (only in Recorded mode)
     */
    bool BeginCompound(const std::string& description);
    
    /**
     * @brief End compound operation
     * @return true if compound operation ended successfully
     */
    bool EndCompound();
    
    /**
     * @brief Check if currently recording compound operation
     * @return true if compound operation is active
     */
    bool IsRecordingCompound() const;
    
    // === Configuration ===
    
    /**
     * @brief Update configuration
     * @param config New configuration
     * @note Some changes (like mode) take effect immediately
     */
    void SetConfig(const Config& config);
    
    /**
     * @brief Get current configuration
     * @return Current configuration
     */
    const Config& GetConfig() const { return config_; }
    
    /**
     * @brief Mark scene as clean (no unsaved changes)
     */
    void MarkClean();
    
    /**
     * @brief Check if scene has unsaved changes
     * @return true if scene is dirty (has unsaved changes)
     */
    bool IsDirty() const;
    
    // === Change Notifications ===
    
    /**
     * @brief Subscribe to change notifications
     * @param callback Function to call on changes
     * @return Subscription ID for unsubscribing
     */
    uint32_t Subscribe(ChangeCallback callback);
    
    /**
     * @brief Unsubscribe from change notifications
     * @param subscription_id ID returned by Subscribe
     */
    void Unsubscribe(uint32_t subscription_id);
    
    // === Statistics and Debugging ===
    
    /**
     * @brief Get memory usage statistics
     * @return Memory usage in bytes
     */
    size_t GetMemoryUsage() const;
    
    /**
     * @brief Clear all state (objects and history)
     * @note This operation cannot be undone
     */
    void Clear();

private:
    // Configuration
    Config config_;
    
    // Object management
    std::unordered_map<ObjectId, std::shared_ptr<OpenGlObject>> objects_;
    ObjectId next_object_id_ = 1;  // 0 reserved as invalid
    mutable std::mutex objects_mutex_;  // Thread-safe object access
    
    // Command system (only used in Recorded mode)
    std::unique_ptr<CommandStack> command_stack_;
    
    // Change notification system
    std::vector<std::pair<uint32_t, ChangeCallback>> observers_;
    uint32_t next_observer_id_ = 1;
    
    // Internal methods
    void NotifyChange(ObjectId id, const std::string& operation);
    void EnsureCommandStack();
    ObjectId GenerateObjectId();
};

// Note: CompoundScope is provided by command_stack.hpp for CommandStack operations

} // namespace quickviz

#endif // QUICKVIZ_SCENE_STATE_HPP