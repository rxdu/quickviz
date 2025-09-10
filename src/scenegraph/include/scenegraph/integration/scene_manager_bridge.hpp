/**
 * @file scene_manager_bridge.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-09-10
 * @brief Bridge integration between SceneState and GlSceneManager
 *
 * Provides seamless integration between the new SceneState management
 * system and the existing GlSceneManager rendering pipeline.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_SCENE_MANAGER_BRIDGE_HPP
#define QUICKVIZ_SCENE_MANAGER_BRIDGE_HPP

#include <memory>
#include <unordered_map>
#include <string>
#include <functional>

#include "scenegraph/state/scene_state.hpp"
#include "gldraw/scene_manager.hpp"

namespace quickviz {

/**
 * @brief Bridge class integrating SceneState with GlSceneManager
 * 
 * This bridge allows existing OpenGL rendering to work seamlessly with
 * the new state management system, enabling undo/redo support and
 * modal operation modes while maintaining backward compatibility.
 * 
 * Key Features:
 * - Synchronizes object registration between SceneState and SceneManager
 * - Translates SceneState operations to SceneManager updates
 * - Provides unified API for both state management and rendering
 * - Maintains performance in Direct mode for real-time applications
 * 
 * Usage Example:
 * ```cpp
 * // Create bridge with existing scene manager
 * auto bridge = std::make_unique<SceneManagerBridge>(scene_manager);
 * 
 * // Switch to recorded mode for editing
 * bridge->SetOperationMode(OperationMode::kRecorded);
 * 
 * // Add object through bridge - automatically tracked in both systems
 * auto object_id = bridge->AddObject("robot", robot_mesh);
 * 
 * // Transform with undo support
 * bridge->SetTransform(object_id, new_transform);
 * bridge->Undo();  // Revert transformation
 * ```
 */
class SceneManagerBridge {
public:
    /**
     * @brief Construct bridge with existing SceneManager
     * @param scene_manager Existing SceneManager to integrate with
     * @param config Optional SceneState configuration
     */
    explicit SceneManagerBridge(std::shared_ptr<SceneManager> scene_manager,
                                const SceneState::Config& config = {});
    
    /**
     * @brief Destructor - ensures proper cleanup
     */
    ~SceneManagerBridge();
    
    // === Operation Mode Management ===
    
    /**
     * @brief Set operation mode for state management
     * @param mode Operation mode (Direct/Immediate/Recorded)
     */
    void SetOperationMode(OperationMode mode);
    
    /**
     * @brief Get current operation mode
     * @return Current operation mode
     */
    OperationMode GetOperationMode() const;
    
    /**
     * @brief Check if undo/redo is supported in current mode
     * @return true if undo/redo is available
     */
    bool SupportsUndo() const;
    
    // === Object Management ===
    
    /**
     * @brief Add object to both SceneState and SceneManager
     * @param name Object name for SceneManager
     * @param object OpenGL object to add
     * @return Object ID for state tracking
     */
    ObjectId AddObject(const std::string& name, 
                      std::shared_ptr<OpenGlObject> object);
    
    /**
     * @brief Remove object from both systems
     * @param id Object ID to remove
     * @return true if object was removed
     */
    bool RemoveObject(ObjectId id);
    
    /**
     * @brief Get object by ID
     * @param id Object ID
     * @return Shared pointer to object, or nullptr if not found
     */
    std::shared_ptr<OpenGlObject> GetObject(ObjectId id) const;
    
    /**
     * @brief Get object by name (SceneManager compatibility)
     * @param name Object name
     * @return Object pointer, or nullptr if not found
     */
    OpenGlObject* GetObjectByName(const std::string& name) const;
    
    /**
     * @brief Get object ID by name
     * @param name Object name
     * @return Object ID, or kInvalidObjectId if not found
     */
    ObjectId GetObjectId(const std::string& name) const;
    
    /**
     * @brief Get object name by ID
     * @param id Object ID
     * @return Object name, or empty string if not found
     */
    std::string GetObjectName(ObjectId id) const;
    
    // === State Operations ===
    
    /**
     * @brief Set transform for an object
     * @param id Object ID
     * @param transform New transformation matrix
     * @return true if operation succeeded
     * 
     * Behavior by mode:
     * - Direct: Immediate update to OpenGL object
     * - Immediate: Update with change tracking
     * - Recorded: Create undoable command
     */
    bool SetTransform(ObjectId id, const glm::mat4& transform);
    
    /**
     * @brief Get transform for an object
     * @param id Object ID
     * @return Current transformation matrix
     */
    glm::mat4 GetTransform(ObjectId id) const;
    
    /**
     * @brief Set visibility for an object
     * @param id Object ID
     * @param visible Visibility state
     * @return true if operation succeeded
     */
    bool SetVisible(ObjectId id, bool visible);
    
    /**
     * @brief Check if object is visible
     * @param id Object ID
     * @return Visibility state
     */
    bool IsVisible(ObjectId id) const;
    
    // === Command Operations ===
    
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
     * @return Description string
     */
    std::string GetUndoDescription() const;
    
    /**
     * @brief Get description of next redo operation
     * @return Description string
     */
    std::string GetRedoDescription() const;
    
    // === Compound Operations ===
    
    /**
     * @brief Begin compound operation for grouping commands
     * @param description Description for the compound operation
     * @return true if compound operation started
     */
    bool BeginCompound(const std::string& description);
    
    /**
     * @brief End compound operation
     * @return true if compound operation ended
     */
    bool EndCompound();
    
    /**
     * @brief Check if currently recording compound operation
     * @return true if compound operation is active
     */
    bool IsRecordingCompound() const;
    
    // === Scene Management ===
    
    /**
     * @brief Get the underlying SceneManager
     * @return Shared pointer to SceneManager
     */
    std::shared_ptr<SceneManager> GetSceneManager() const { 
        return scene_manager_; 
    }
    
    /**
     * @brief Get the SceneState
     * @return Pointer to SceneState
     */
    SceneState* GetSceneState() { return scene_state_.get(); }
    const SceneState* GetSceneState() const { return scene_state_.get(); }
    
    /**
     * @brief Synchronize state from SceneManager to SceneState
     * 
     * Useful for importing existing scene objects into state management
     */
    void SynchronizeFromSceneManager();
    
    /**
     * @brief Clear all objects from both systems
     */
    void Clear();
    
    // === Statistics ===
    
    /**
     * @brief Get command stack statistics
     * @return Statistics object
     */
    CommandStack::Statistics GetCommandStatistics() const;
    
    /**
     * @brief Get memory usage
     * @return Memory usage in bytes
     */
    size_t GetMemoryUsage() const;
    
    /**
     * @brief Get all registered object names
     * @return Vector of object names currently registered in the bridge
     */
    std::vector<std::string> GetAllObjectNames() const;

private:
    // Core components
    std::shared_ptr<SceneManager> scene_manager_;
    std::unique_ptr<SceneState> scene_state_;
    
    // Bidirectional mapping between names and IDs
    std::unordered_map<std::string, ObjectId> name_to_id_;
    std::unordered_map<ObjectId, std::string> id_to_name_;
    
    // Internal synchronization methods
    void SyncObjectToSceneManager(ObjectId id, const std::string& name);
    void SyncObjectFromSceneManager(const std::string& name);
    void OnStateChanged(ObjectId id, const std::string& operation);
    
    // Change notification subscription
    uint32_t change_subscription_id_ = 0;
};

/**
 * @brief RAII helper for compound operations
 * 
 * Ensures compound operations are properly ended even if exceptions occur
 * 
 * Usage:
 * ```cpp
 * {
 *     CompoundOperation compound(bridge, "Multiple transforms");
 *     bridge->SetTransform(id1, transform1);
 *     bridge->SetTransform(id2, transform2);
 *     // Automatically ends compound on scope exit
 * }
 * ```
 */
class CompoundOperation {
public:
    explicit CompoundOperation(SceneManagerBridge* bridge, 
                              const std::string& description)
        : bridge_(bridge), active_(false) {
        if (bridge_) {
            active_ = bridge_->BeginCompound(description);
        }
    }
    
    ~CompoundOperation() {
        if (active_ && bridge_) {
            bridge_->EndCompound();
        }
    }
    
    // Disable copy/move
    CompoundOperation(const CompoundOperation&) = delete;
    CompoundOperation& operator=(const CompoundOperation&) = delete;
    
private:
    SceneManagerBridge* bridge_;
    bool active_;
};

} // namespace quickviz

#endif // QUICKVIZ_SCENE_MANAGER_BRIDGE_HPP