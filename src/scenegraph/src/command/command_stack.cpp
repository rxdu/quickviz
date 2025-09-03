/**
 * @file command_stack.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-09-03
 * @brief Implementation of CommandStack for undo/redo management
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "scenegraph/command/command_stack.hpp"

#include <algorithm>
#include <stdexcept>

namespace quickviz {

// === CommandStack Implementation ===

CommandStack::CommandStack() : CommandStack(Config{}) {
}

CommandStack::CommandStack(const Config& config) 
    : config_(config) {
    UpdateStatistics();
}

CommandStack::~CommandStack() {
    Clear();
}

void CommandStack::Execute(std::unique_ptr<Command> command) {
    if (!command) {
        throw std::invalid_argument("Cannot execute null command");
    }
    
    // Check if we should track this command
    if (!ShouldTrackCommand(*command)) {
        command->Execute();
        return;
    }
    
    // If recording compound operation, add to compound instead
    if (IsRecordingCompound()) {
        command->Execute();
        recording_compound_->AddCommand(std::move(command));
        return;
    }
    
    try {
        // Execute the command
        command->Execute();
        
        // Clear redo stack (new timeline branch created)
        redo_stack_.clear();
        
        // Add to undo stack
        undo_stack_.push_back(std::move(command));
        
        // Update position tracking
        current_position_++;
        
        // Enforce limits
        EnforceMemoryLimit();
        EnforceCommandLimit();
        
        // Update stats and notify
        stats_.total_commands_executed++;
        UpdateStatistics();
        NotifyObservers();
        
    } catch (...) {
        // If command execution failed, don't add to history
        throw;
    }
}

bool CommandStack::Undo() {
    if (!CanUndo()) {
        return false;
    }
    
    auto command = std::move(undo_stack_.back());
    undo_stack_.pop_back();
    
    try {
        command->Undo();
        
        // Move to redo stack
        redo_stack_.push_back(std::move(command));
        
        // Update position
        current_position_--;
        
        // Update stats and notify
        stats_.undo_count++;
        UpdateStatistics();
        NotifyObservers();
        
        return true;
        
    } catch (...) {
        // If undo failed, put command back and re-throw
        undo_stack_.push_back(std::move(command));
        throw;
    }
}

bool CommandStack::Redo() {
    if (!CanRedo()) {
        return false;
    }
    
    auto command = std::move(redo_stack_.back());
    redo_stack_.pop_back();
    
    try {
        command->Execute();
        
        // Move back to undo stack
        undo_stack_.push_back(std::move(command));
        
        // Update position
        current_position_++;
        
        // Update stats and notify
        stats_.redo_count++;
        UpdateStatistics();
        NotifyObservers();
        
        return true;
        
    } catch (...) {
        // If redo failed, put command back and re-throw
        redo_stack_.push_back(std::move(command));
        throw;
    }
}

bool CommandStack::CanUndo() const {
    return !undo_stack_.empty() && 
           (undo_stack_.empty() || undo_stack_.back()->CanUndo());
}

bool CommandStack::CanRedo() const {
    return !redo_stack_.empty();
}

std::string CommandStack::GetUndoDescription() const {
    if (CanUndo()) {
        return "Undo " + undo_stack_.back()->GetDescription();
    }
    return "";
}

std::string CommandStack::GetRedoDescription() const {
    if (CanRedo()) {
        return "Redo " + redo_stack_.back()->GetDescription();
    }
    return "";
}

CommandStack::Statistics CommandStack::GetStatistics() const {
    UpdateStatistics();
    return stats_;
}

void CommandStack::SetMaxCommands(size_t max_commands) {
    config_.max_commands = max_commands;
    EnforceCommandLimit();
    UpdateStatistics();
    NotifyObservers();
}

void CommandStack::SetMemoryLimit(size_t limit_bytes) {
    config_.memory_limit_bytes = limit_bytes;
    EnforceMemoryLimit();
    UpdateStatistics();
    NotifyObservers();
}

void CommandStack::SetAutoCompress(bool enable) {
    config_.auto_compress = enable;
    if (enable) {
        CompressHistory();
    }
}

void CommandStack::SetTrackPersistentOnly(bool persistent_only) {
    config_.track_persistent_only = persistent_only;
}

void CommandStack::Clear() {
    undo_stack_.clear();
    redo_stack_.clear();
    current_position_ = 0;
    clean_position_ = 0;
    
    UpdateStatistics();
    NotifyObservers();
}

size_t CommandStack::CompressHistory() {
    if (!config_.auto_compress || undo_stack_.size() < 2) {
        return 0;
    }
    
    size_t commands_removed = 0;
    std::deque<std::unique_ptr<Command>> compressed_stack;
    
    // Process commands pairwise for compression opportunities
    for (size_t i = 0; i < undo_stack_.size(); ++i) {
        if (i + 1 < undo_stack_.size() && 
            CanCompress(*undo_stack_[i], *undo_stack_[i + 1])) {
            
            // Compress two commands into one
            auto compressed = CompressCommands(
                std::move(undo_stack_[i]), 
                std::move(undo_stack_[i + 1])
            );
            
            if (compressed) {
                compressed_stack.push_back(std::move(compressed));
                commands_removed++;
                i++; // Skip next command since we compressed it
            } else {
                compressed_stack.push_back(std::move(undo_stack_[i]));
            }
        } else {
            compressed_stack.push_back(std::move(undo_stack_[i]));
        }
    }
    
    undo_stack_ = std::move(compressed_stack);
    stats_.commands_compressed += commands_removed;
    
    UpdateStatistics();
    NotifyObservers();
    
    return commands_removed;
}

void CommandStack::MarkClean() {
    clean_position_ = current_position_;
}

bool CommandStack::IsClean() const {
    return clean_position_ == current_position_;
}

uint32_t CommandStack::Subscribe(ChangeCallback callback) {
    uint32_t id = next_observer_id_++;
    observers_.emplace_back(id, std::move(callback));
    return id;
}

void CommandStack::Unsubscribe(uint32_t subscription_id) {
    observers_.erase(
        std::remove_if(observers_.begin(), observers_.end(),
            [subscription_id](const auto& pair) {
                return pair.first == subscription_id;
            }),
        observers_.end()
    );
}

void CommandStack::ExecuteWithoutHistory(std::unique_ptr<Command> command) {
    if (!command) {
        throw std::invalid_argument("Cannot execute null command");
    }
    
    command->Execute();
    // Command is not added to history
}

void CommandStack::BeginCompound(const std::string& description) {
    if (IsRecordingCompound()) {
        throw std::runtime_error("Already recording compound command");
    }
    
    recording_compound_ = std::make_unique<CompoundCommand>(description);
}

bool CommandStack::EndCompound() {
    if (!IsRecordingCompound()) {
        return false;
    }
    
    auto compound = std::move(recording_compound_);
    
    // Only add compound if it has sub-commands
    if (compound->GetCommandCount() > 0) {
        // Mark compound as executed since sub-commands were already executed
        // We need to set the executed count manually
        compound->executed_count_ = compound->GetCommandCount();
        
        // Add to undo stack without executing
        undo_stack_.push_back(std::move(compound));
        current_position_++;
        
        EnforceMemoryLimit();
        EnforceCommandLimit();
        
        UpdateStatistics();
        NotifyObservers();
        
        return true;
    }
    
    return false;
}

bool CommandStack::IsRecordingCompound() const {
    return recording_compound_ != nullptr;
}

// === Private Methods ===

void CommandStack::EnforceMemoryLimit() {
    if (config_.memory_limit_bytes == 0) {
        return; // No limit
    }
    
    size_t current_usage = CalculateMemoryUsage();
    
    while (current_usage > config_.memory_limit_bytes && !undo_stack_.empty()) {
        // Remove oldest command
        current_usage -= undo_stack_.front()->GetMemorySize();
        undo_stack_.pop_front();
        stats_.commands_discarded++;
        
        // Adjust clean position if needed
        if (clean_position_ > 0) {
            clean_position_--;
        }
    }
}

void CommandStack::EnforceCommandLimit() {
    if (config_.max_commands == 0) {
        return; // No limit
    }
    
    while (undo_stack_.size() > config_.max_commands) {
        undo_stack_.pop_front();
        stats_.commands_discarded++;
        
        // Adjust clean position if needed  
        if (clean_position_ > 0) {
            clean_position_--;
        }
    }
}

void CommandStack::UpdateStatistics() const {
    stats_.current_undo_depth = undo_stack_.size();
    stats_.current_redo_depth = redo_stack_.size();
    stats_.memory_usage_bytes = CalculateMemoryUsage();
}

void CommandStack::NotifyObservers() {
    UpdateStatistics();
    for (const auto& [id, callback] : observers_) {
        try {
            callback(stats_);
        } catch (...) {
            // Ignore observer exceptions to maintain stack integrity
        }
    }
}

size_t CommandStack::CalculateMemoryUsage() const {
    size_t total = sizeof(*this);
    
    for (const auto& command : undo_stack_) {
        total += command->GetMemorySize();
    }
    
    for (const auto& command : redo_stack_) {
        total += command->GetMemorySize();
    }
    
    if (recording_compound_) {
        total += recording_compound_->GetMemorySize();
    }
    
    return total;
}

bool CommandStack::ShouldTrackCommand(const Command& command) const {
    if (config_.track_persistent_only) {
        return command.ModifiesPersistentState();
    }
    return true;
}

bool CommandStack::CanCompress(const Command& a, const Command& b) const {
    // Basic compression: same command type with similar descriptions
    // More sophisticated compression would be implemented in derived classes
    return a.GetDescription() == b.GetDescription() &&
           a.ModifiesPersistentState() == b.ModifiesPersistentState();
}

std::unique_ptr<Command> CommandStack::CompressCommands(
    std::unique_ptr<Command> a, 
    std::unique_ptr<Command> b) const {
    
    // Basic implementation - just return the later command
    // More sophisticated compression would merge the operations
    return std::move(b);
}

// === CompoundScope Implementation ===

CompoundScope::CompoundScope(CommandStack& stack, const std::string& description)
    : stack_(stack) {
    stack_.BeginCompound(description);
}

CompoundScope::~CompoundScope() {
    if (!completed_) {
        stack_.EndCompound();
    }
}

} // namespace quickviz