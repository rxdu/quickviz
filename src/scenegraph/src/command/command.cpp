/**
 * @file command.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-09-03
 * @brief Implementation of command pattern classes
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "scenegraph/command/command.hpp"

#include <stdexcept>
#include <numeric>

namespace quickviz {

// === CompoundCommand Implementation ===

CompoundCommand::CompoundCommand(const std::string& description)
    : description_(description) {
}

void CompoundCommand::AddCommand(std::unique_ptr<Command> command) {
    if (!command) {
        throw std::invalid_argument("Cannot add null command to CompoundCommand");
    }
    commands_.push_back(std::move(command));
}

void CompoundCommand::Reserve(size_t count) {
    commands_.reserve(count);
}

void CompoundCommand::Execute() {
    executed_count_ = 0;
    
    try {
        // Execute all commands in order
        for (size_t i = 0; i < commands_.size(); ++i) {
            commands_[i]->Execute();
            executed_count_ = i + 1;
        }
    } catch (...) {
        // If any command fails, undo all previously executed commands
        // Execute undo in reverse order
        for (size_t i = executed_count_; i > 0; --i) {
            try {
                commands_[i - 1]->Undo();
            } catch (...) {
                // If undo fails, we're in an inconsistent state
                // This is a serious error that should be logged
                // but we can't do much else here
            }
        }
        executed_count_ = 0;
        throw; // Re-throw original exception
    }
}

void CompoundCommand::Undo() {
    if (executed_count_ == 0) {
        throw std::runtime_error("Cannot undo CompoundCommand that was never executed");
    }
    
    // Undo in reverse order of execution
    for (size_t i = executed_count_; i > 0; --i) {
        try {
            commands_[i - 1]->Undo();
        } catch (const std::exception& e) {
            // If any undo fails, we have a serious problem
            // Try to continue with remaining undos, but track the failure
            throw std::runtime_error("CompoundCommand undo failed at step " + 
                                   std::to_string(i) + ": " + e.what());
        }
    }
    
    executed_count_ = 0;
}

size_t CompoundCommand::GetMemorySize() const {
    // Base size plus size of all contained commands
    size_t total_size = sizeof(*this);
    
    for (const auto& command : commands_) {
        total_size += command->GetMemorySize();
    }
    
    return total_size;
}

std::string CompoundCommand::GetDescription() const {
    if (!description_.empty()) {
        return description_;
    }
    
    // Generate description from sub-commands if none provided
    if (commands_.empty()) {
        return "Empty Compound Command";
    } else if (commands_.size() == 1) {
        return commands_[0]->GetDescription();
    } else {
        return "Compound Command (" + std::to_string(commands_.size()) + " operations)";
    }
}

bool CompoundCommand::CanUndo() const {
    // Can undo if all executed commands can be undone
    for (size_t i = 0; i < executed_count_; ++i) {
        if (!commands_[i]->CanUndo()) {
            return false;
        }
    }
    return executed_count_ > 0;
}

bool CompoundCommand::ModifiesPersistentState() const {
    // Modifies persistent state if any sub-command does
    for (const auto& command : commands_) {
        if (command->ModifiesPersistentState()) {
            return true;
        }
    }
    return false;
}

// === NoOpCommand Implementation ===

NoOpCommand::NoOpCommand(const std::string& description)
    : description_(description) {
}

} // namespace quickviz