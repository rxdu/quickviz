/**
 * @file command.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-09-03
 * @brief Command pattern interface for undo/redo operations
 *
 * Industrial-strength command pattern following Unity/Blender paradigm.
 * Provides foundation for stateful operations with reversible execution.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_COMMAND_HPP
#define QUICKVIZ_COMMAND_HPP

#include <memory>
#include <string>
#include <vector>
#include <cstddef>

namespace quickviz {

/**
 * @brief Abstract base class for all reversible operations
 * 
 * Command pattern implementation supporting:
 * - Execute/Undo semantics for state management
 * - Memory usage tracking for history limits
 * - Operation descriptions for UI display
 * - Exception safety guarantees
 * 
 * Design follows industrial practices from:
 * - Unity EditorGUI.BeginChangeCheck pattern
 * - Blender Operator system
 * - Photoshop History panel
 * 
 * Usage:
 * ```cpp
 * auto cmd = std::make_unique<TransformCommand>(object, new_transform);
 * command_stack.Execute(std::move(cmd));  // Do + record for undo
 * command_stack.Undo();                   // Reverse operation
 * ```
 */
class Command {
public:
    virtual ~Command() = default;

    /**
     * @brief Execute the command (forward operation)
     * 
     * Implementations must:
     * - Be idempotent (safe to call multiple times)
     * - Handle partial failure gracefully
     * - Maintain strong exception safety
     * 
     * @throws std::runtime_error if operation cannot be completed
     */
    virtual void Execute() = 0;

    /**
     * @brief Undo the command (reverse operation)
     * 
     * Requirements:
     * - Must perfectly reverse Execute() effects
     * - Should not fail if Execute() succeeded previously
     * - Must be callable even after object state changes
     * 
     * @throws std::runtime_error if undo cannot be completed
     */
    virtual void Undo() = 0;

    /**
     * @brief Get memory footprint for history management
     * 
     * Used by CommandStack to:
     * - Enforce memory limits on undo history
     * - Prioritize which commands to keep/discard
     * - Display memory usage in debugging tools
     * 
     * @return Approximate memory usage in bytes
     * @note Should include size of captured state data
     */
    virtual size_t GetMemorySize() const = 0;

    /**
     * @brief Get human-readable operation description
     * 
     * Used for:
     * - History panel display ("Undo Transform", "Redo Delete Points")
     * - Logging and debugging output
     * - User interface feedback
     * 
     * @return Brief description suitable for UI display
     * @note Should be localization-ready
     */
    virtual std::string GetDescription() const = 0;

    /**
     * @brief Check if command can be safely undone
     * 
     * Default implementation returns true. Override for commands that:
     * - Depend on external resources that might be deleted
     * - Have time-sensitive preconditions
     * - Require specific application state to undo
     * 
     * @return true if Undo() is safe to call, false otherwise
     */
    virtual bool CanUndo() const { return true; }

    /**
     * @brief Check if command modifies persistent state
     * 
     * Used to determine:
     * - Whether scene needs saving
     * - If temporary operations should be recorded
     * - Cache invalidation requirements
     * 
     * @return true if command changes persistent data
     * @note Temporary operations (camera moves, selection) return false
     */
    virtual bool ModifiesPersistentState() const { return true; }

protected:
    /**
     * @brief Protected constructor - commands must be created via derived classes
     */
    Command() = default;

    // Prevent copying - commands should be unique operations
    Command(const Command&) = delete;
    Command& operator=(const Command&) = delete;

    // Allow moving for efficient storage in containers
    Command(Command&&) = default;
    Command& operator=(Command&&) = default;
};

/**
 * @brief Compound command for grouping related operations
 * 
 * Enables atomic execution of multiple commands:
 * - All succeed or all fail (transaction semantics)
 * - Single undo operation reverses entire group
 * - Useful for complex operations like "Delete Selected Points"
 * 
 * Example:
 * ```cpp
 * auto compound = std::make_unique<CompoundCommand>("Delete Selection");
 * compound->AddCommand(std::make_unique<UpdateSelectionCommand>({}));
 * compound->AddCommand(std::make_unique<DeletePointsCommand>(indices));  
 * compound->AddCommand(std::make_unique<RefreshRenderCommand>());
 * ```
 */
class CompoundCommand : public Command {
public:
    /**
     * @brief Construct compound command with description
     * @param description Human-readable name for the compound operation
     */
    explicit CompoundCommand(const std::string& description);

    /**
     * @brief Add sub-command to the compound operation
     * @param command Command to add (ownership transferred)
     * @note Commands are executed in the order they were added
     */
    void AddCommand(std::unique_ptr<Command> command);

    /**
     * @brief Reserve space for known number of sub-commands
     * @param count Expected number of commands (optimization hint)
     */
    void Reserve(size_t count);

    /**
     * @brief Get number of sub-commands
     * @return Count of contained commands
     */
    size_t GetCommandCount() const { return commands_.size(); }

    // Command interface
    void Execute() override;
    void Undo() override;
    size_t GetMemorySize() const override;
    std::string GetDescription() const override;
    bool CanUndo() const override;
    bool ModifiesPersistentState() const override;

private:
    std::string description_;
    std::vector<std::unique_ptr<Command>> commands_;
    
    // Track execution state for proper undo sequencing
    size_t executed_count_ = 0;
    
    // Allow CommandStack to set execution state for compound operations
    friend class CommandStack;
};

/**
 * @brief No-operation command for testing and placeholder operations
 * 
 * Useful for:
 * - Unit testing command infrastructure
 * - Placeholder operations during development
 * - Measuring command system overhead
 */
class NoOpCommand : public Command {
public:
    explicit NoOpCommand(const std::string& description = "No Operation");

    void Execute() override {} // Intentionally empty
    void Undo() override {}    // Intentionally empty
    size_t GetMemorySize() const override { return sizeof(*this); }
    std::string GetDescription() const override { return description_; }
    bool ModifiesPersistentState() const override { return false; }

private:
    std::string description_;
};

} // namespace quickviz

#endif // QUICKVIZ_COMMAND_HPP