/**
 * @file scene_state.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-09-03
 * @brief Implementation of SceneState for modal scene management
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "scenegraph/state/scene_state.hpp"

#include <algorithm>
#include <stdexcept>

namespace quickviz {

// === Internal Command Implementations ===

/**
 * @brief Command for object transform operations
 */
class TransformCommand : public Command {
public:
    TransformCommand(ObjectId id, const glm::mat4& new_transform, 
                    std::shared_ptr<OpenGlObject> object)
        : id_(id), new_transform_(new_transform), object_(object) {
        if (object) {
            old_transform_ = object->GetTransform();
        }
    }

    void Execute() override {
        if (auto obj = object_.lock()) {
            obj->SetTransform(new_transform_);
        }
    }

    void Undo() override {
        if (auto obj = object_.lock()) {
            obj->SetTransform(old_transform_);
        }
    }

    size_t GetMemorySize() const override {
        return sizeof(*this);
    }

    std::string GetDescription() const override {
        return "Transform Object " + std::to_string(id_);
    }

private:
    ObjectId id_;
    glm::mat4 old_transform_;
    glm::mat4 new_transform_;
    std::weak_ptr<OpenGlObject> object_;
};

/**
 * @brief Command for object visibility operations
 */
class VisibilityCommand : public Command {
public:
    VisibilityCommand(ObjectId id, bool new_visible, 
                     std::shared_ptr<OpenGlObject> object)
        : id_(id), new_visible_(new_visible), object_(object) {
        if (object) {
            old_visible_ = object->IsVisible();
        }
    }

    void Execute() override {
        if (auto obj = object_.lock()) {
            obj->SetVisible(new_visible_);
        }
    }

    void Undo() override {
        if (auto obj = object_.lock()) {
            obj->SetVisible(old_visible_);
        }
    }

    size_t GetMemorySize() const override {
        return sizeof(*this);
    }

    std::string GetDescription() const override {
        return (new_visible_ ? "Show" : "Hide") + std::string(" Object ") + std::to_string(id_);
    }

    bool ModifiesPersistentState() const override {
        return true;
    }

private:
    ObjectId id_;
    bool old_visible_;
    bool new_visible_;
    std::weak_ptr<OpenGlObject> object_;
};

/**
 * @brief Command for object registration operations
 */
class UnregisterCommand : public Command {
public:
    UnregisterCommand(ObjectId id, std::shared_ptr<OpenGlObject> object)
        : id_(id), object_(object) {}

    void Execute() override {
        // Unregistration happens in SceneState, we just track the operation
        executed_ = true;
    }

    void Undo() override {
        // Cannot undo unregistration without re-registering
        // This would need to be handled by SceneState
        throw std::runtime_error("Cannot undo object unregistration - object must be re-registered");
    }

    size_t GetMemorySize() const override {
        return sizeof(*this) + (object_ ? sizeof(*object_) : 0);
    }

    std::string GetDescription() const override {
        return "Remove Object " + std::to_string(id_);
    }

    bool CanUndo() const override {
        return false; // Cannot safely undo unregistration
    }

private:
    ObjectId id_;
    std::shared_ptr<OpenGlObject> object_;
    bool executed_ = false;
};

// === SceneState Implementation ===

SceneState::SceneState() : SceneState(Config{}) {}

SceneState::SceneState(const Config& config) : config_(config) {
    if (config_.mode == OperationMode::kRecorded) {
        EnsureCommandStack();
    }
}

SceneState::~SceneState() {
    Clear();
}

void SceneState::SetMode(OperationMode mode) {
    config_.mode = mode;
    
    if (mode == OperationMode::kRecorded) {
        EnsureCommandStack();
    } else {
        // In non-recorded modes, we don't need the command stack
        command_stack_.reset();
    }
}

ObjectId SceneState::RegisterObject(std::shared_ptr<OpenGlObject> object) {
    if (!object) {
        throw std::invalid_argument("Cannot register null object");
    }
    
    std::lock_guard<std::mutex> lock(objects_mutex_);
    
    ObjectId id = GenerateObjectId();
    objects_[id] = object;
    
    // Registration is always immediate - no command wrapping
    NotifyChange(id, "Register");
    
    return id;
}

bool SceneState::UnregisterObject(ObjectId id) {
    std::shared_ptr<OpenGlObject> object;
    
    {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        auto it = objects_.find(id);
        if (it == objects_.end()) {
            return false;
        }
        object = it->second;
    }
    
    // Handle according to mode
    switch (config_.mode) {
        case OperationMode::kDirect: {
            std::lock_guard<std::mutex> lock(objects_mutex_);
            objects_.erase(id);
            NotifyChange(id, "Unregister");
            return true;
        }
        
        case OperationMode::kImmediate: {
            std::lock_guard<std::mutex> lock(objects_mutex_);
            objects_.erase(id);
            NotifyChange(id, "Unregister");
            return true;
        }
        
        case OperationMode::kRecorded: {
            // Note: Unregistration is tricky to make undoable
            // For now, we'll make it immediate but record the action
            auto cmd = std::make_unique<UnregisterCommand>(id, object);
            
            {
                std::lock_guard<std::mutex> lock(objects_mutex_);
                objects_.erase(id);
            }
            
            if (command_stack_) {
                command_stack_->ExecuteWithoutHistory(std::move(cmd));
            }
            
            NotifyChange(id, "Unregister");
            return true;
        }
    }
    
    return false;
}

std::shared_ptr<OpenGlObject> SceneState::GetObject(ObjectId id) const {
    std::lock_guard<std::mutex> lock(objects_mutex_);
    auto it = objects_.find(id);
    return (it != objects_.end()) ? it->second : nullptr;
}

bool SceneState::HasObject(ObjectId id) const {
    std::lock_guard<std::mutex> lock(objects_mutex_);
    return objects_.find(id) != objects_.end();
}

size_t SceneState::GetObjectCount() const {
    std::lock_guard<std::mutex> lock(objects_mutex_);
    return objects_.size();
}

bool SceneState::SetTransform(ObjectId id, const glm::mat4& transform) {
    auto object = GetObject(id);
    if (!object) {
        return false;
    }
    
    switch (config_.mode) {
        case OperationMode::kDirect:
            object->SetTransform(transform);
            return true;
            
        case OperationMode::kImmediate:
            object->SetTransform(transform);
            NotifyChange(id, "Transform");
            return true;
            
        case OperationMode::kRecorded:
            if (command_stack_) {
                auto cmd = std::make_unique<TransformCommand>(id, transform, object);
                command_stack_->Execute(std::move(cmd));
                NotifyChange(id, "Transform");
                return true;
            }
            return false;
    }
    
    return false;
}

glm::mat4 SceneState::GetTransform(ObjectId id) const {
    auto object = GetObject(id);
    return object ? object->GetTransform() : glm::mat4(1.0f);
}

bool SceneState::SetVisible(ObjectId id, bool visible) {
    auto object = GetObject(id);
    if (!object) {
        return false;
    }
    
    switch (config_.mode) {
        case OperationMode::kDirect:
            object->SetVisible(visible);
            return true;
            
        case OperationMode::kImmediate:
            object->SetVisible(visible);
            NotifyChange(id, visible ? "Show" : "Hide");
            return true;
            
        case OperationMode::kRecorded:
            if (command_stack_) {
                auto cmd = std::make_unique<VisibilityCommand>(id, visible, object);
                command_stack_->Execute(std::move(cmd));
                NotifyChange(id, visible ? "Show" : "Hide");
                return true;
            }
            return false;
    }
    
    return false;
}

bool SceneState::IsVisible(ObjectId id) const {
    auto object = GetObject(id);
    return object ? object->IsVisible() : false;
}

bool SceneState::ExecuteCommand(std::unique_ptr<Command> command) {
    if (config_.mode != OperationMode::kRecorded || !command_stack_) {
        return false;
    }
    
    command_stack_->Execute(std::move(command));
    return true;
}

bool SceneState::Undo() {
    if (config_.mode != OperationMode::kRecorded || !command_stack_) {
        return false;
    }
    
    return command_stack_->Undo();
}

bool SceneState::Redo() {
    if (config_.mode != OperationMode::kRecorded || !command_stack_) {
        return false;
    }
    
    return command_stack_->Redo();
}

bool SceneState::CanUndo() const {
    return (config_.mode == OperationMode::kRecorded && command_stack_) ?
           command_stack_->CanUndo() : false;
}

bool SceneState::CanRedo() const {
    return (config_.mode == OperationMode::kRecorded && command_stack_) ?
           command_stack_->CanRedo() : false;
}

std::string SceneState::GetUndoDescription() const {
    return (config_.mode == OperationMode::kRecorded && command_stack_) ?
           command_stack_->GetUndoDescription() : "";
}

std::string SceneState::GetRedoDescription() const {
    return (config_.mode == OperationMode::kRecorded && command_stack_) ?
           command_stack_->GetRedoDescription() : "";
}

CommandStack::Statistics SceneState::GetCommandStatistics() const {
    return (config_.mode == OperationMode::kRecorded && command_stack_) ?
           command_stack_->GetStatistics() : CommandStack::Statistics{};
}

bool SceneState::BeginCompound(const std::string& description) {
    if (config_.mode != OperationMode::kRecorded || !command_stack_) {
        return false;
    }
    
    command_stack_->BeginCompound(description);
    return true;
}

bool SceneState::EndCompound() {
    if (config_.mode != OperationMode::kRecorded || !command_stack_) {
        return false;
    }
    
    return command_stack_->EndCompound();
}

bool SceneState::IsRecordingCompound() const {
    return (config_.mode == OperationMode::kRecorded && command_stack_) ?
           command_stack_->IsRecordingCompound() : false;
}

void SceneState::SetConfig(const Config& config) {
    config_ = config;
    
    if (config_.mode == OperationMode::kRecorded) {
        EnsureCommandStack();
        if (command_stack_) {
            command_stack_->SetMaxCommands(config_.max_commands);
            command_stack_->SetMemoryLimit(config_.memory_limit_bytes);
            command_stack_->SetAutoCompress(config_.auto_compress);
            command_stack_->SetTrackPersistentOnly(config_.track_persistent_only);
        }
    } else {
        command_stack_.reset();
    }
}

void SceneState::MarkClean() {
    if (config_.mode == OperationMode::kRecorded && command_stack_) {
        command_stack_->MarkClean();
    }
}

bool SceneState::IsDirty() const {
    return (config_.mode == OperationMode::kRecorded && command_stack_) ?
           !command_stack_->IsClean() : false;
}

uint32_t SceneState::Subscribe(ChangeCallback callback) {
    if (!config_.enable_change_notifications) {
        return 0;
    }
    
    uint32_t id = next_observer_id_++;
    observers_.emplace_back(id, std::move(callback));
    return id;
}

void SceneState::Unsubscribe(uint32_t subscription_id) {
    observers_.erase(
        std::remove_if(observers_.begin(), observers_.end(),
            [subscription_id](const auto& pair) {
                return pair.first == subscription_id;
            }),
        observers_.end()
    );
}

size_t SceneState::GetMemoryUsage() const {
    size_t total = sizeof(*this);
    
    {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        total += objects_.size() * (sizeof(ObjectId) + sizeof(std::shared_ptr<OpenGlObject>));
    }
    
    if (command_stack_) {
        total += command_stack_->GetStatistics().memory_usage_bytes;
    }
    
    return total;
}

void SceneState::Clear() {
    {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        objects_.clear();
        next_object_id_ = 1;
    }
    
    if (command_stack_) {
        command_stack_->Clear();
    }
    
    observers_.clear();
}

void SceneState::NotifyChange(ObjectId id, const std::string& operation) {
    if (!config_.enable_change_notifications) {
        return;
    }
    
    for (const auto& [subscription_id, callback] : observers_) {
        try {
            callback(id, operation);
        } catch (...) {
            // Ignore observer exceptions to maintain state integrity
        }
    }
}

void SceneState::EnsureCommandStack() {
    if (!command_stack_) {
        CommandStack::Config cmd_config;
        cmd_config.max_commands = config_.max_commands;
        cmd_config.memory_limit_bytes = config_.memory_limit_bytes;
        cmd_config.auto_compress = config_.auto_compress;
        cmd_config.track_persistent_only = config_.track_persistent_only;
        
        command_stack_ = std::make_unique<CommandStack>(cmd_config);
    }
}

ObjectId SceneState::GenerateObjectId() {
    return next_object_id_++;
}

// Note: CompoundScope implementation is in command_stack.cpp

} // namespace quickviz