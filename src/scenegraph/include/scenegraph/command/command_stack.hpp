/**
 * @file command_stack.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-09-03
 * @brief Command stack for undo/redo history management
 *
 * Professional-grade undo/redo system with memory management,
 * following patterns from Unity, Blender, and Photoshop.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_COMMAND_STACK_HPP
#define QUICKVIZ_COMMAND_STACK_HPP

#include <deque>
#include <memory>
#include <functional>
#include <string>
#include <vector>

#include "scenegraph/command/command.hpp"

namespace quickviz {

/**
 * @brief Undo/redo command history manager with memory limits
 * 
 * Features:
 * - Configurable history depth and memory limits
 * - Automatic cleanup of old commands when limits exceeded
 * - Thread-safe operation (single writer, multiple readers)
 * - Command compression to optimize memory usage
 * - Statistics and debugging support
 * 
 * Industrial patterns:
 * - Unity: EditorGUI undo stack with memory pressure handling
 * - Blender: Operator history with configurable limits  
 * - Photoshop: History panel with step limits and memory usage
 * 
 * Usage:
 * ```cpp
 * CommandStack stack;
 * stack.SetMaxCommands(100);
 * stack.SetMemoryLimit(512 * 1024 * 1024);  // 512 MB
 * 
 * stack.Execute(std::make_unique<TransformCommand>(...));
 * if (stack.CanUndo()) stack.Undo();
 * if (stack.CanRedo()) stack.Redo();
 * ```
 */
class CommandStack {
public:
    /**
     * @brief Configuration for command stack behavior
     */
    struct Config {
        size_t max_commands = 100;              // Maximum commands in history
        size_t memory_limit_bytes = 256 * 1024 * 1024;  // 256 MB default
        bool auto_compress = true;              // Compress similar commands
        bool track_persistent_only = false;    // Only track persistent operations
        float compression_ratio = 0.8f;        // Target compression ratio
    };

    /**
     * @brief Statistics about command stack state
     */
    struct Statistics {
        size_t total_commands_executed = 0;    // Lifetime command count
        size_t undo_count = 0;                 // Times Undo() called
        size_t redo_count = 0;                 // Times Redo() called
        size_t current_undo_depth = 0;         // Commands available for undo
        size_t current_redo_depth = 0;         // Commands available for redo
        size_t memory_usage_bytes = 0;         // Current memory usage
        size_t commands_compressed = 0;        // Commands optimized away
        size_t commands_discarded = 0;         // Commands removed due to limits
    };

    /**
     * @brief Command stack change notification callback
     */
    using ChangeCallback = std::function<void(const Statistics&)>;

public:
    /**
     * @brief Construct command stack with default configuration
     */
    CommandStack();

    /**
     * @brief Construct command stack with custom configuration
     * @param config Configuration parameters
     */
    explicit CommandStack(const Config& config);

    /**
     * @brief Destructor - ensures proper cleanup
     */
    ~CommandStack();

    // === Command Execution ===

    /**
     * @brief Execute command and add to undo history
     * 
     * Process:
     * 1. Execute the command
     * 2. Clear any redo history (new branch created)
     * 3. Add to undo stack
     * 4. Apply memory/count limits
     * 5. Notify observers
     * 
     * @param command Command to execute (ownership transferred)
     * @throws std::runtime_error if command execution fails
     */
    void Execute(std::unique_ptr<Command> command);

    /**
     * @brief Undo most recent command
     * 
     * @return true if undo was successful, false if no commands to undo
     * @throws std::runtime_error if undo operation fails
     */
    bool Undo();

    /**
     * @brief Redo most recently undone command
     * 
     * @return true if redo was successful, false if no commands to redo  
     * @throws std::runtime_error if redo operation fails
     */
    bool Redo();

    // === State Queries ===

    /**
     * @brief Check if undo is available
     * @return true if at least one command can be undone
     */
    bool CanUndo() const;

    /**
     * @brief Check if redo is available
     * @return true if at least one command can be redone
     */
    bool CanRedo() const;

    /**
     * @brief Get description of next undo operation
     * @return Description string, or empty if no undo available
     */
    std::string GetUndoDescription() const;

    /**
     * @brief Get description of next redo operation  
     * @return Description string, or empty if no redo available
     */
    std::string GetRedoDescription() const;

    /**
     * @brief Get current statistics
     * @return Statistics structure with current state
     */
    Statistics GetStatistics() const;

    // === Configuration ===

    /**
     * @brief Set maximum number of commands to keep
     * @param max_commands Maximum commands (0 = unlimited)
     */
    void SetMaxCommands(size_t max_commands);

    /**
     * @brief Set memory limit for command history
     * @param limit_bytes Maximum memory usage (0 = unlimited)
     */
    void SetMemoryLimit(size_t limit_bytes);

    /**
     * @brief Enable/disable automatic command compression
     * @param enable True to enable compression optimization
     */
    void SetAutoCompress(bool enable);

    /**
     * @brief Set whether to track only persistent operations
     * @param persistent_only True to ignore temporary operations
     */
    void SetTrackPersistentOnly(bool persistent_only);

    // === History Management ===

    /**
     * @brief Clear all undo/redo history
     * 
     * Useful for:
     * - Starting fresh editing session
     * - After major state changes (file load)
     * - Memory pressure situations
     */
    void Clear();

    /**
     * @brief Compress command history to reduce memory usage
     * 
     * Optimization strategies:
     * - Merge sequential transform operations
     * - Remove redundant state changes
     * - Compact similar operations
     * 
     * @return Number of commands removed through compression
     */
    size_t CompressHistory();

    /**
     * @brief Mark current position as "clean" (saved state)
     * 
     * Used to track:
     * - Whether document needs saving
     * - Clean/dirty state for UI indicators
     * - Automatic backup triggers
     */
    void MarkClean();

    /**
     * @brief Check if current state matches last clean position
     * @return true if no commands executed since MarkClean()
     */
    bool IsClean() const;

    // === Observers ===

    /**
     * @brief Subscribe to command stack changes
     * @param callback Function called after stack modifications
     * @return Subscription ID for unsubscribing
     */
    uint32_t Subscribe(ChangeCallback callback);

    /**
     * @brief Unsubscribe from change notifications
     * @param subscription_id ID returned from Subscribe()
     */
    void Unsubscribe(uint32_t subscription_id);

    // === Advanced Operations ===

    /**
     * @brief Execute command without recording for undo
     * 
     * Used for:
     * - Internal operations that shouldn't be undoable
     * - Temporary state changes during operations
     * - System-initiated updates
     * 
     * @param command Command to execute (ownership transferred)
     */
    void ExecuteWithoutHistory(std::unique_ptr<Command> command);

    /**
     * @brief Begin compound operation
     * 
     * All commands executed until EndCompound() will be grouped
     * into a single undoable operation.
     * 
     * @param description Name for the compound operation
     */
    void BeginCompound(const std::string& description);

    /**
     * @brief End compound operation and add to history
     * 
     * @return true if compound operation was created and added
     */
    bool EndCompound();

    /**
     * @brief Check if currently recording compound operation
     * @return true if BeginCompound() was called without EndCompound()
     */
    bool IsRecordingCompound() const;

private:
    // Configuration
    Config config_;

    // Command storage
    std::deque<std::unique_ptr<Command>> undo_stack_;
    std::deque<std::unique_ptr<Command>> redo_stack_;

    // Statistics
    mutable Statistics stats_;

    // Clean state tracking
    size_t clean_position_ = 0;  // Position when MarkClean() was called
    size_t current_position_ = 0;  // Current position in history

    // Compound command recording
    std::unique_ptr<CompoundCommand> recording_compound_;
    
    // Observer pattern
    std::vector<std::pair<uint32_t, ChangeCallback>> observers_;
    uint32_t next_observer_id_ = 1;

    // Internal methods
    void EnforceMemoryLimit();
    void EnforceCommandLimit();
    void UpdateStatistics() const;
    void NotifyObservers();
    size_t CalculateMemoryUsage() const;
    bool ShouldTrackCommand(const Command& command) const;

    // Command compression helpers
    bool CanCompress(const Command& a, const Command& b) const;
    std::unique_ptr<Command> CompressCommands(
        std::unique_ptr<Command> a, 
        std::unique_ptr<Command> b) const;
};

/**
 * @brief RAII helper for compound operations
 * 
 * Automatically calls BeginCompound() in constructor and EndCompound()
 * in destructor, ensuring proper cleanup even with exceptions.
 * 
 * Usage:
 * ```cpp
 * {
 *     CompoundScope scope(stack, "Delete Selection");
 *     stack.Execute(std::make_unique<ClearSelectionCommand>());
 *     stack.Execute(std::make_unique<DeletePointsCommand>(indices));
 *     // Compound automatically completed when scope exits
 * }
 * ```
 */
class CompoundScope {
public:
    CompoundScope(CommandStack& stack, const std::string& description);
    ~CompoundScope();

    // Prevent copying/moving to avoid double-completion
    CompoundScope(const CompoundScope&) = delete;
    CompoundScope& operator=(const CompoundScope&) = delete;
    CompoundScope(CompoundScope&&) = delete;
    CompoundScope& operator=(CompoundScope&&) = delete;

private:
    CommandStack& stack_;
    bool completed_ = false;
};

} // namespace quickviz

#endif // QUICKVIZ_COMMAND_STACK_HPP