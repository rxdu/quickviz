/**
 * @file test_command.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-09-03
 * @brief Unit tests for command pattern implementation
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "scenegraph/command/command.hpp"
#include "scenegraph/command/command_stack.hpp"

namespace quickviz {
namespace test {

// === Test Command Implementation ===

class TestCommand : public Command {
public:
    TestCommand(const std::string& description, int* target, int new_value)
        : description_(description), target_(target), new_value_(new_value), old_value_(*target) {}

    void Execute() override {
        old_value_ = *target_;
        *target_ = new_value_;
        executed_ = true;
    }

    void Undo() override {
        *target_ = old_value_;
        executed_ = false;
    }

    size_t GetMemorySize() const override {
        return sizeof(*this);
    }

    std::string GetDescription() const override {
        return description_;
    }

    bool WasExecuted() const { return executed_; }

private:
    std::string description_;
    int* target_;
    int new_value_;
    int old_value_;
    bool executed_ = false;
};

class FailingCommand : public Command {
public:
    FailingCommand(bool fail_execute = true, bool fail_undo = false)
        : fail_execute_(fail_execute), fail_undo_(fail_undo) {}

    void Execute() override {
        if (fail_execute_) {
            throw std::runtime_error("Execute failed");
        }
        executed_ = true;
    }

    void Undo() override {
        if (fail_undo_) {
            throw std::runtime_error("Undo failed");
        }
        executed_ = false;
    }

    size_t GetMemorySize() const override { return sizeof(*this); }
    std::string GetDescription() const override { return "Failing Command"; }

    bool WasExecuted() const { return executed_; }

private:
    bool fail_execute_;
    bool fail_undo_;
    bool executed_ = false;
};

// === Command Tests ===

class CommandTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_value_ = 0;
    }

    int test_value_;
};

TEST_F(CommandTest, BasicExecution) {
    auto cmd = std::make_unique<TestCommand>("Set to 5", &test_value_, 5);
    
    EXPECT_EQ(test_value_, 0);
    EXPECT_FALSE(cmd->WasExecuted());
    
    cmd->Execute();
    EXPECT_EQ(test_value_, 5);
    EXPECT_TRUE(cmd->WasExecuted());
}

TEST_F(CommandTest, BasicUndo) {
    auto cmd = std::make_unique<TestCommand>("Set to 10", &test_value_, 10);
    
    cmd->Execute();
    EXPECT_EQ(test_value_, 10);
    
    cmd->Undo();
    EXPECT_EQ(test_value_, 0);
    EXPECT_FALSE(cmd->WasExecuted());
}

TEST_F(CommandTest, CommandDescription) {
    auto cmd = std::make_unique<TestCommand>("Test Description", &test_value_, 1);
    EXPECT_EQ(cmd->GetDescription(), "Test Description");
}

TEST_F(CommandTest, CommandMemorySize) {
    auto cmd = std::make_unique<TestCommand>("Memory Test", &test_value_, 1);
    EXPECT_GT(cmd->GetMemorySize(), 0);
}

// === CompoundCommand Tests ===

TEST_F(CommandTest, CompoundCommandBasic) {
    int second_value = 0;
    
    auto compound = std::make_unique<CompoundCommand>("Compound Test");
    compound->AddCommand(std::make_unique<TestCommand>("First", &test_value_, 1));
    compound->AddCommand(std::make_unique<TestCommand>("Second", &second_value, 2));
    
    EXPECT_EQ(compound->GetCommandCount(), 2);
    
    compound->Execute();
    EXPECT_EQ(test_value_, 1);
    EXPECT_EQ(second_value, 2);
    
    compound->Undo();
    EXPECT_EQ(test_value_, 0);
    EXPECT_EQ(second_value, 0);
}

TEST_F(CommandTest, CompoundCommandFailure) {
    int second_value = 0;
    
    auto compound = std::make_unique<CompoundCommand>("Failing Compound");
    compound->AddCommand(std::make_unique<TestCommand>("First", &test_value_, 1));
    compound->AddCommand(std::make_unique<FailingCommand>(true, false));  // Fails on execute
    compound->AddCommand(std::make_unique<TestCommand>("Third", &second_value, 3));
    
    // Execute should fail and undo first command
    EXPECT_THROW(compound->Execute(), std::runtime_error);
    EXPECT_EQ(test_value_, 0);  // Should be undone
    EXPECT_EQ(second_value, 0);  // Should never be executed
}

// === NoOpCommand Tests ===

TEST_F(CommandTest, NoOpCommand) {
    auto cmd = std::make_unique<NoOpCommand>("Test NoOp");
    
    // Should not affect anything
    cmd->Execute();
    EXPECT_EQ(test_value_, 0);
    
    cmd->Undo();
    EXPECT_EQ(test_value_, 0);
    
    EXPECT_EQ(cmd->GetDescription(), "Test NoOp");
    EXPECT_FALSE(cmd->ModifiesPersistentState());
}

// === CommandStack Tests ===

class CommandStackTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_value_ = 0;
        stack_ = std::make_unique<CommandStack>();
    }

    int test_value_;
    std::unique_ptr<CommandStack> stack_;
};

TEST_F(CommandStackTest, BasicExecution) {
    EXPECT_FALSE(stack_->CanUndo());
    EXPECT_FALSE(stack_->CanRedo());
    
    stack_->Execute(std::make_unique<TestCommand>("Set to 5", &test_value_, 5));
    
    EXPECT_EQ(test_value_, 5);
    EXPECT_TRUE(stack_->CanUndo());
    EXPECT_FALSE(stack_->CanRedo());
    EXPECT_EQ(stack_->GetUndoDescription(), "Undo Set to 5");
}

TEST_F(CommandStackTest, UndoRedo) {
    stack_->Execute(std::make_unique<TestCommand>("Set to 10", &test_value_, 10));
    EXPECT_EQ(test_value_, 10);
    
    EXPECT_TRUE(stack_->Undo());
    EXPECT_EQ(test_value_, 0);
    EXPECT_FALSE(stack_->CanUndo());
    EXPECT_TRUE(stack_->CanRedo());
    EXPECT_EQ(stack_->GetRedoDescription(), "Redo Set to 10");
    
    EXPECT_TRUE(stack_->Redo());
    EXPECT_EQ(test_value_, 10);
    EXPECT_TRUE(stack_->CanUndo());
    EXPECT_FALSE(stack_->CanRedo());
}

TEST_F(CommandStackTest, MultipleCommands) {
    stack_->Execute(std::make_unique<TestCommand>("Set to 1", &test_value_, 1));
    stack_->Execute(std::make_unique<TestCommand>("Set to 2", &test_value_, 2));
    stack_->Execute(std::make_unique<TestCommand>("Set to 3", &test_value_, 3));
    
    EXPECT_EQ(test_value_, 3);
    
    EXPECT_TRUE(stack_->Undo());
    EXPECT_EQ(test_value_, 2);
    
    EXPECT_TRUE(stack_->Undo());
    EXPECT_EQ(test_value_, 1);
    
    EXPECT_TRUE(stack_->Undo());
    EXPECT_EQ(test_value_, 0);
    
    EXPECT_FALSE(stack_->CanUndo());
}

TEST_F(CommandStackTest, RedoClearedOnNewCommand) {
    stack_->Execute(std::make_unique<TestCommand>("Set to 1", &test_value_, 1));
    stack_->Execute(std::make_unique<TestCommand>("Set to 2", &test_value_, 2));
    
    // Undo to enable redo
    stack_->Undo();
    EXPECT_TRUE(stack_->CanRedo());
    
    // New command should clear redo stack
    stack_->Execute(std::make_unique<TestCommand>("Set to 3", &test_value_, 3));
    EXPECT_FALSE(stack_->CanRedo());
    EXPECT_EQ(test_value_, 3);
}

TEST_F(CommandStackTest, Statistics) {
    auto stats = stack_->GetStatistics();
    EXPECT_EQ(stats.total_commands_executed, 0);
    EXPECT_EQ(stats.current_undo_depth, 0);
    
    stack_->Execute(std::make_unique<TestCommand>("Test", &test_value_, 1));
    
    stats = stack_->GetStatistics();
    EXPECT_EQ(stats.total_commands_executed, 1);
    EXPECT_EQ(stats.current_undo_depth, 1);
    EXPECT_EQ(stats.current_redo_depth, 0);
    EXPECT_GT(stats.memory_usage_bytes, 0);
}

TEST_F(CommandStackTest, MaxCommandsLimit) {
    stack_->SetMaxCommands(2);
    
    stack_->Execute(std::make_unique<TestCommand>("Cmd 1", &test_value_, 1));
    stack_->Execute(std::make_unique<TestCommand>("Cmd 2", &test_value_, 2));
    stack_->Execute(std::make_unique<TestCommand>("Cmd 3", &test_value_, 3));
    
    auto stats = stack_->GetStatistics();
    EXPECT_EQ(stats.current_undo_depth, 2);  // Should be limited to 2
    EXPECT_EQ(stats.commands_discarded, 1);  // First command discarded
    
    // Should not be able to undo back to original value (0)
    stack_->Undo();
    stack_->Undo();
    EXPECT_FALSE(stack_->CanUndo());
    EXPECT_NE(test_value_, 0);  // Can't get back to original
}

TEST_F(CommandStackTest, CompoundOperations) {
    int second_value = 0;
    
    stack_->BeginCompound("Multi-step operation");
    stack_->Execute(std::make_unique<TestCommand>("Step 1", &test_value_, 5));
    stack_->Execute(std::make_unique<TestCommand>("Step 2", &second_value, 10));
    EXPECT_TRUE(stack_->EndCompound());
    
    EXPECT_EQ(test_value_, 5);
    EXPECT_EQ(second_value, 10);
    
    // Single undo should reverse both operations
    EXPECT_TRUE(stack_->Undo());
    EXPECT_EQ(test_value_, 0);
    EXPECT_EQ(second_value, 0);
    
    auto stats = stack_->GetStatistics();
    EXPECT_EQ(stats.current_undo_depth, 0);
}

TEST_F(CommandStackTest, CompoundScope) {
    int second_value = 0;
    
    {
        CompoundScope scope(*stack_, "Scoped Operation");
        stack_->Execute(std::make_unique<TestCommand>("A", &test_value_, 1));
        stack_->Execute(std::make_unique<TestCommand>("B", &second_value, 2));
        // Compound automatically completed when scope exits
    }
    
    EXPECT_EQ(test_value_, 1);
    EXPECT_EQ(second_value, 2);
    
    // Should be a single compound operation
    EXPECT_TRUE(stack_->Undo());
    EXPECT_EQ(test_value_, 0);
    EXPECT_EQ(second_value, 0);
}

TEST_F(CommandStackTest, CleanState) {
    EXPECT_TRUE(stack_->IsClean());
    
    stack_->Execute(std::make_unique<TestCommand>("Dirty", &test_value_, 1));
    EXPECT_FALSE(stack_->IsClean());
    
    stack_->MarkClean();
    EXPECT_TRUE(stack_->IsClean());
    
    stack_->Execute(std::make_unique<TestCommand>("Dirty again", &test_value_, 2));
    EXPECT_FALSE(stack_->IsClean());
    
    stack_->Undo();
    EXPECT_TRUE(stack_->IsClean());  // Back to clean state
}

TEST_F(CommandStackTest, ExecuteWithoutHistory) {
    stack_->ExecuteWithoutHistory(std::make_unique<TestCommand>("No history", &test_value_, 5));
    
    EXPECT_EQ(test_value_, 5);
    EXPECT_FALSE(stack_->CanUndo());  // Not recorded in history
    
    auto stats = stack_->GetStatistics();
    EXPECT_EQ(stats.total_commands_executed, 0);  // Not counted
}

// === Observer Tests ===

TEST_F(CommandStackTest, ChangeNotification) {
    bool notified = false;
    CommandStack::Statistics last_stats;
    
    auto subscription = stack_->Subscribe([&](const CommandStack::Statistics& stats) {
        notified = true;
        last_stats = stats;
    });
    
    stack_->Execute(std::make_unique<TestCommand>("Notify test", &test_value_, 1));
    
    EXPECT_TRUE(notified);
    EXPECT_EQ(last_stats.total_commands_executed, 1);
    
    stack_->Unsubscribe(subscription);
    
    notified = false;
    stack_->Execute(std::make_unique<TestCommand>("No notify", &test_value_, 2));
    EXPECT_FALSE(notified);  // Should not be notified after unsubscribe
}

} // namespace test
} // namespace quickviz