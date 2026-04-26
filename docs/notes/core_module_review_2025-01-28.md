# Core Module Review and Improvement Considerations

**Date:** January 28, 2025  
**Context:** Comprehensive review of core module to eliminate duplication and improve design quality

## Issues Identified

### 1. Singleton Anti-Pattern Usage
- **AsyncEventDispatcher**: Uses singleton pattern, making testing difficult
- **BufferRegistry**: Singleton pattern, but actively used by multiple widgets
- **Impact**: Hidden dependencies, testing difficulties, inflexible design

### 2. Event System Architecture
- **Current State**: Three event systems exist
  - `EventDispatcher` (modern, instance-based, synchronous)
  - `AsyncEventDispatcher` (singleton, asynchronous, queue-based)
  - Both have separate emitters
- **Usage Patterns**:
  - Sync: UI input handling, immediate response needed, event consumption
  - Async: Background processing, decoupled systems, producer-consumer pattern
- **Decision**: Keep separate systems due to different use cases and threading models

### 3. Design Pattern Discussion: Singleton vs Instance-Based

#### Singleton Pattern (Current AsyncEventDispatcher)
**Pros:**
- Global access, no instance passing needed
- Automatic lifetime management
- Single processing thread guaranteed

**Cons:**
- Hidden dependencies, testing nightmare
- No isolation between components
- Inflexible (can't have multiple async systems)
- Thread coupling with rigid validation

#### Instance-Based Design (Proposed)
**Pros:**
- Testable (fresh instances per test)
- Flexible (multiple dispatchers for different purposes)
- Clear dependencies via constructor injection
- Isolated (separate event buses per component)
- Configurable per instance

**Cons:**
- Need to manage lifetime and pass instances
- Slightly more complex setup

### 4. Other Issues Found
- **Test Organization**: Duplication between `test/` (integration) and `test/unit_test/` (unit tests)
- **Font Resources**: Large embedded font headers (247KB+) in core includes
- **ThreadSafeQueue**: Wrong namespace (`xmotion` vs `quickviz`)

## Architectural Questions for AsyncEventDispatcher

### Scope Options:
1. **Global**: Single instance across entire application
2. **Per-Component**: Each widget/panel has own instance
3. **Per-Subsystem**: File I/O, rendering, networking each have separate instance

### Lifetime Management:
1. **Application-managed**: Top-level application owns instances
2. **Component-managed**: Individual components create/manage own instances
3. **Factory/Container**: Dependency injection container manages lifecycle

### Threading Models:
1. **Dedicated Thread**: Each instance has own background thread
2. **Shared Thread Pool**: Multiple instances share thread pool
3. **Configurable**: Instance can choose threading strategy

## Proposed Implementation Strategy

### Phase 1: AsyncEventDispatcher Improvements
- Remove singleton pattern → instance-based design
- Add priority support (align with EventDispatcher)
- Add optional consumption mechanism
- Improve thread management (dedicated background thread)
- Consistent handler signatures (bool return for consumption)
- Better error handling (remove rigid thread validation)

### Phase 2: BufferRegistry Refactoring
- Convert from singleton to dependency injection
- Update all widgets that depend on BufferRegistry
- Maintain backward compatibility during transition

### Phase 3: Resource Organization
- Move font resources from `core/include/core/fonts/` to `resources/fonts/`
- Update build system to handle resource files
- Fix namespace issues in ThreadSafeQueue

### Phase 4: Test Cleanup
- Clarify test organization (integration vs unit)
- Remove or consolidate duplicate test files
- Improve test documentation

## Decision Points Needed

1. **AsyncEventDispatcher Scope**: How should instances be scoped and managed?
2. **BufferRegistry Migration**: Gradual transition vs big-bang approach?
3. **Resource Location**: `resources/` vs `third_party/` for fonts?
4. **Threading Strategy**: Dedicated threads vs shared pool for async dispatchers?

## Implementation Priority

1. **Critical**: BufferRegistry singleton removal (affects multiple widgets, blocks current usage)
2. **Important**: AsyncEventDispatcher improvements (valuable infrastructure for GUI applications)
3. **Medium**: Font resource organization (cleanup, not functional)
4. **Low**: Test organization (maintenance, not user-facing)

## Updated Context

AsyncEventDispatcher is an important infrastructure component for GUI/visualization applications, providing decoupled event processing capabilities that are commonly needed in interactive applications. While currently functional, improvements would benefit future GUI development.