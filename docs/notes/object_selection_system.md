# Object Selection System

## Overview

QuickViz provides two complementary selection systems for 3D objects:
1. **Point Cloud Selection** - Pixel-perfect selection of individual points using GPU ID-buffer technique
2. **Object Selection** - Ray-casting based selection of general 3D objects using bounding box intersection

This document explains how the object selection system works and how to add selection support to custom renderable objects.

## Architecture

### Selection Flow

```
Mouse Click → Screen Coordinates → Ray Generation → Ray-Box Intersection → Object Selection → Visual Feedback
```

1. **Mouse Input Capture**: When the user clicks in the 3D view, the screen coordinates are captured
2. **Ray Generation**: Screen coordinates are unprojected to create a ray in world space
3. **Intersection Testing**: The ray is tested against bounding boxes of all selectable objects
4. **Selection Update**: The closest intersecting object becomes selected
5. **Visual Feedback**: Selected object is highlighted (e.g., color change)

### Key Components

#### GlSceneManager API

```cpp
// Select object at screen position
std::string SelectObjectAt(float screen_x, float screen_y);

// Clear current selection  
void ClearObjectSelection();

// Get selected object name
const std::string& GetSelectedObjectName() const;

// Manual highlight control
void SetObjectHighlight(const std::string& name, bool highlighted);

// Selection callback
using ObjectSelectionCallback = std::function<void(const std::string&)>;
void SetObjectSelectionCallback(ObjectSelectionCallback callback);
```

#### OpenGlObject Interface Extensions

```cpp
// Check if object supports selection
virtual bool SupportsSelection() const { return false; }

// Get axis-aligned bounding box in world space
virtual std::pair<glm::vec3, glm::vec3> GetBoundingBox() const;

// Visual feedback when selected
virtual void SetHighlighted(bool highlighted);
```

## Implementation Details

### Ray Generation

The ray generation process converts 2D screen coordinates to a 3D ray in world space:

```cpp
// Convert mouse to NDC (-1 to 1)
float x_ndc = (2.0f * mouse_x) / window_width - 1.0f;
float y_ndc = 1.0f - (2.0f * mouse_y) / window_height;

// Unproject through inverse matrices
glm::vec4 ray_clip(x_ndc, y_ndc, -1.0f, 1.0f);
glm::vec4 ray_eye = inverse(projection) * ray_clip;
glm::vec4 ray_world = inverse(view) * ray_eye;

// Ray from camera position
ray.origin = camera_position;
ray.direction = normalize(ray_world);
```

### Coordinate System Transformation

QuickViz uses Z-up coordinate system internally but transforms to OpenGL's Y-up system for rendering. The selection system handles this by:

1. **Ray stays in OpenGL space**: The ray is generated in the transformed (Y-up) space where the camera operates
2. **Bounding boxes are transformed**: Object bounds are transformed from Z-up to Y-up before intersection testing

```cpp
if (use_coord_transform_) {
  // Transform all 8 corners of the bounding box
  glm::vec3 corners[8] = { /* ... */ };
  
  glm::vec3 new_min(FLT_MAX);
  glm::vec3 new_max(-FLT_MAX);
  
  for (auto& corner : corners) {
    glm::vec4 transformed = coord_transform_ * glm::vec4(corner, 1.0f);
    new_min = glm::min(new_min, glm::vec3(transformed));
    new_max = glm::max(new_max, glm::vec3(transformed));
  }
  
  // Use transformed bounds for intersection
  min_bounds = new_min;
  max_bounds = new_max;
}
```

### Ray-Box Intersection

The system uses the slab method for ray-AABB (Axis-Aligned Bounding Box) intersection:

```cpp
glm::vec3 inv_dir = 1.0f / ray.direction;
glm::vec3 t_min = (min_bounds - ray.origin) * inv_dir;
glm::vec3 t_max = (max_bounds - ray.origin) * inv_dir;

glm::vec3 t1 = glm::min(t_min, t_max);
glm::vec3 t2 = glm::max(t_min, t_max);

float t_near = glm::max(glm::max(t1.x, t1.y), t1.z);
float t_far = glm::min(glm::min(t2.x, t2.y), t2.z);

// Ray intersects if t_near <= t_far and t_far >= 0
if (t_near <= t_far && t_far >= 0) {
  float distance = t_near >= 0 ? t_near : t_far;
  // Object is hit at this distance
}
```

## Adding Selection to Custom Objects

### Step 1: Enable Selection Support

```cpp
class MyObject : public OpenGlObject {
public:
  bool SupportsSelection() const override { 
    return true; 
  }
};
```

### Step 2: Provide Bounding Box

```cpp
std::pair<glm::vec3, glm::vec3> GetBoundingBox() const override {
  // Return min and max corners of your object's bounds
  glm::vec3 min_bounds = position_ - glm::vec3(width_/2, height_/2, depth_/2);
  glm::vec3 max_bounds = position_ + glm::vec3(width_/2, height_/2, depth_/2);
  return {min_bounds, max_bounds};
}
```

### Step 3: Implement Visual Feedback (Optional)

```cpp
void SetHighlighted(bool highlighted) override {
  if (highlighted) {
    // Save original appearance and apply highlight
    original_color_ = current_color_;
    current_color_ = glm::vec3(1.0f, 1.0f, 0.0f); // Yellow
  } else {
    // Restore original appearance
    current_color_ = original_color_;
  }
}
```

## Example: Sphere Selection

Here's how the Sphere class implements selection:

```cpp
class Sphere : public OpenGlObject {
private:
  glm::vec3 center_ = glm::vec3(0.0f);
  float radius_ = 1.0f;
  glm::vec3 color_ = glm::vec3(0.7f, 0.7f, 0.9f);
  glm::vec3 original_color_;
  bool is_highlighted_ = false;

public:
  bool SupportsSelection() const override { 
    return true; 
  }
  
  std::pair<glm::vec3, glm::vec3> GetBoundingBox() const override {
    glm::vec3 half_extents(radius_, radius_, radius_);
    return {center_ - half_extents, center_ + half_extents};
  }
  
  void SetHighlighted(bool highlighted) override {
    is_highlighted_ = highlighted;
    if (highlighted) {
      original_color_ = color_;
      color_ = glm::vec3(1.0f, 1.0f, 0.0f); // Yellow highlight
    } else {
      color_ = original_color_;
    }
  }
};
```

## Usage Example

```cpp
// Create scene manager
auto scene = std::make_shared<GlSceneManager>("3D View");

// Add selectable objects
auto sphere = std::make_unique<Sphere>(glm::vec3(0, 0, 1), 0.5f);
scene->AddOpenGLObject("sphere1", std::move(sphere));

// Set selection callback
scene->SetObjectSelectionCallback([](const std::string& name) {
  if (!name.empty()) {
    std::cout << "Selected: " << name << std::endl;
  } else {
    std::cout << "Selection cleared" << std::endl;
  }
});

// In your input handling
void OnMouseClick(float x, float y) {
  std::string selected = scene->SelectObjectAt(x, y);
}

void OnRightClick() {
  scene->ClearObjectSelection();
}
```

## Performance Considerations

### Bounding Box Quality
- Tight bounding boxes improve selection accuracy
- Loose bounds may cause false positives
- For complex shapes, consider using oriented bounding boxes (OBB) or bounding spheres

### Optimization Strategies
1. **Spatial Partitioning**: Use octrees or BVH for scenes with many objects
2. **LOD Selection**: Use simpler bounds for distant objects
3. **Caching**: Cache transformed bounding boxes if coordinate transform is static
4. **Early Rejection**: Skip objects outside view frustum

### Limitations
- Current implementation uses AABB which may not fit irregular shapes well
- No support for selecting occluded objects (always selects closest)
- No multi-selection support (single object at a time)

## Comparison with Point Cloud Selection

| Feature | Object Selection | Point Cloud Selection |
|---------|-----------------|----------------------|
| Method | Ray-box intersection | GPU ID-buffer |
| Precision | Bounding box accuracy | Pixel-perfect |
| Performance | O(n) objects | O(1) after ID render |
| Memory | Minimal | Requires extra framebuffer |
| Best For | Meshes, primitives | Large point datasets |

## Future Enhancements

Potential improvements to the selection system:

1. **Multi-selection**: Support selecting multiple objects with box selection or Ctrl+Click
2. **Selection Modes**: Add different modes like wireframe, outline glow, or transparency
3. **Hierarchical Selection**: Select object groups or scene graph nodes
4. **Pick Info**: Return intersection point, normal, and distance
5. **Custom Intersection**: Allow objects to implement custom ray intersection tests
6. **Selection Filters**: Filter selectable objects by type, layer, or custom predicate

## Troubleshooting

### Object Not Selectable
- Verify `SupportsSelection()` returns true
- Check bounding box is non-zero size
- Ensure object is added to scene manager
- Verify object is within camera view

### Wrong Object Selected
- Check bounding boxes don't overlap
- Verify coordinate transformation is correct
- Ensure bounding box accurately represents object

### Selection Not Working
- Check mouse coordinates are in window space
- Verify camera matrices are valid
- Enable debug output to check ray generation
- Test with known good object (e.g., sphere at origin)