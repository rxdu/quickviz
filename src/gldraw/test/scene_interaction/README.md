# Scene Interaction Selection Tests

This directory contains modular selection tests for different renderable object types in QuickViz. The test structure has been refactored from a single large file into focused, maintainable test applications.

## Test Structure

### Shared Utilities
- **`selection_test_utils.hpp/cpp`**: Common utilities, UI components, and test base classes
  - `SelectionInfoPanel`: UI panel showing selection details
  - `SelectionDemoPanel`: Enhanced scene panel with selection handling
  - `SelectionTestApp`: Base class for all selection tests
  - `TestObjectFactory`: Factory methods for creating test objects
  - `TestHelpers`: Utility functions for test setup

### Individual Object Type Tests

#### `test_sphere_selection.cpp`
Tests sphere object selection functionality:
- Individual sphere highlighting (yellow glow)
- Multi-selection support 
- Performance with 50+ spheres
- Different sphere sizes and configurations (grid, layered, random)

#### `test_point_cloud_selection.cpp` 
Tests individual point selection within point clouds:
- GPU-based pixel-perfect point picking
- Point highlighting through layer system
- Multiple point cloud patterns (grid, spiral, cluster, gradient)
- Dense point cloud performance testing

#### `test_line_strip_selection.cpp` 
Tests LineStrip selection functionality (newly implemented):
- LineStrip highlighting (yellow color, 2x line width)
- Bounding box calculation for different line patterns
- Various line types: geometric patterns, mathematical curves, robot paths
- Complex polylines and boundary contours

#### `test_comprehensive_selection.cpp`
Comprehensive test combining all selection types:
- Mixed object scene (spheres, point clouds, line strips)
- Multi-selection across different object types
- Realistic robotics navigation scenario
- Selection mode filtering (P/O/H keys)

### Legacy Test
#### `test_object_selection.cpp` 
Original comprehensive test (preserved for backward compatibility)

## Usage

### Building Tests
```bash
cd build
make test_sphere_selection
make test_point_cloud_selection  
make test_line_strip_selection
make test_comprehensive_selection
```

### Running Tests
```bash
# Test specific object types
./bin/test_sphere_selection
./bin/test_point_cloud_selection
./bin/test_line_strip_selection

# Test all object types together
./bin/test_comprehensive_selection
```

## Controls

All tests share common interaction patterns:

### Mouse Controls
- **Left Click**: Select object/point
- **Ctrl+Shift+Click**: Add to multi-selection
- **Ctrl+Alt+Click**: Toggle selection
- **Ctrl+Right Click**: Clear all selections

### Keyboard Shortcuts
- **P**: Point selection mode (point clouds only)
- **O**: Object selection mode (spheres, lines only)  
- **H**: Hybrid selection mode (everything) [DEFAULT]
- **C**: Clear selection

### Visual Feedback
- **Spheres**: Yellow highlight with original size
- **Points**: Yellow highlight with increased size
- **LineStrips**: Yellow color with 2x line width

## Selection System Features

### Supported Object Types
- ✅ **Spheres**: Object-level selection with highlighting
- ✅ **Point Clouds**: Individual point selection via GPU picking
- ✅ **LineStrips**: Object-level selection with visual feedback (NEW)
- 🔄 **Meshes**: Planned (triangle-accurate selection)
- 🔄 **Cylinders**: Planned (refinement needed)

### Core Functionality
- **GPU ID-Buffer Selection**: Pixel-perfect object identification
- **Multi-Selection**: Ctrl+Shift+Click to build selection sets
- **Selection Modes**: Filter by object type (points/objects/hybrid)
- **Visual Feedback**: Consistent highlighting across object types
- **Bounding Box Calculation**: Accurate bounds for all selectable objects

## Adding New Selection Tests

To add a new renderable type to the selection test suite:

1. **Create test file**: `test_<type>_selection.cpp`
2. **Inherit from SelectionTestApp**: 
   ```cpp
   class NewTypeSelectionTest : public SelectionTestApp {
     // Implement SetupTestObjects() and description methods
   };
   ```
3. **Add to CMakeLists.txt**:
   ```cmake
   add_executable(test_new_type_selection test_new_type_selection.cpp)
   target_link_libraries(test_new_type_selection PRIVATE selection_test_utils)
   ```
4. **Use TestObjectFactory helpers** for consistent test object creation

## Design Principles

### Modularity
- Each test focuses on one object type for maintainability
- Shared utilities eliminate code duplication
- Clear separation of concerns (UI, test logic, object creation)

### Reusability  
- Base classes and utilities support easy test extension
- Factory methods provide consistent object creation patterns
- Common interaction patterns across all tests

### Performance
- Individual tests allow focused performance analysis
- Comprehensive test validates system integration
- Scalable test object generation (10s to 1000s of objects)

### User Experience
- Consistent controls and visual feedback
- Clear on-screen instructions
- Console output for debugging and verification

## Implementation Details

### Selection Architecture
The selection system uses a two-tier approach:
1. **GPU ID-Buffer Rendering**: Objects render with unique ID colors for pixel-perfect picking
2. **CPU Ray Intersection**: Fallback and validation using bounding box tests

### Visual Feedback Implementation
Each object type implements the OpenGlObject selection interface:
- `SupportsSelection()` → returns `true` for selectable objects
- `SetHighlighted(bool)` → applies visual feedback
- `GetBoundingBox()` → returns world-space bounds
- `SupportsIdRendering()` → enables GPU picking

### Multi-Selection Management
The SelectionManager maintains:
- Current selection state (SelectionResult)
- Multi-selection collection (MultiSelection) 
- Selection callbacks for UI updates
- Selection mode filtering logic

## Future Enhancements

### Planned Object Types
- **Mesh Selection**: Triangle-accurate selection for area editing
- **Cylinder Selection**: Enhanced highlighting with proper SetHighlighted() implementation  
- **Text3D Selection**: Clickable labels and annotations
- **Arrow Selection**: Directional indicator selection

### Advanced Features
- **Selection Groups**: Hierarchical selection management
- **Selection History**: Undo/redo for selection operations
- **Selection Persistence**: Save/load selection states
- **Custom Selection Tools**: Box select, lasso select, proximity select