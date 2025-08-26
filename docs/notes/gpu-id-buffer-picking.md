# GPU ID-Buffer Point Picking Design

*Design decision for QuickViz point cloud selection system*
*Date: August 26, 2025*

## Problem Statement

The current ray-casting point selection system has fundamental limitations that prevent robust point cloud interaction:

### Issues with Current Ray-Casting Approach

**Occlusion Problems**:
- Ray can pass through multiple points at different depths
- Selection algorithm may pick background points instead of visible foreground points
- No automatic handling of point visibility from camera perspective

**Camera Sensitivity**:
- Ray calculations depend heavily on camera position and orientation
- Small camera movements can drastically change selection behavior
- Ray origin often positioned outside point cloud bounds, causing distance calculation issues

**Tolerance Ambiguity**:
- Difficult to determine appropriate tolerance values for different point densities
- Too small: miss intended points; too large: ambiguous selections
- Tolerance needs vary with camera distance and point cloud scale

**Performance Limitations**:
- O(n) brute force search through all points for each click
- No spatial optimization for large point clouds
- CPU-bound algorithm cannot leverage GPU parallel processing

**User Experience Issues**:
- Selected point may not match what user visually sees on screen
- Inconsistent behavior across different viewing angles
- Poor precision for dense point clouds

## Proposed Solution: GPU ID-Buffer Picking

### Core Concept

**GPU ID-Buffer Picking** is the industry-standard approach used by professional visualization tools (ParaView, CloudCompare, MeshLab, etc.). The technique renders the scene off-screen with each point assigned a unique color ID, then reads back the pixel color at mouse position to determine which point was clicked.

### Technical Approach

#### 1. **ID Encoding Strategy**
- Encode point indices in RGB color values (24-bit → ~16M unique IDs)
- ID = `(R << 16) | (G << 8) | B`
- Point index = ID - 1 (reserve ID 0 for background)
- Handle point clouds with >16M points via tiling or 32-bit RGBA encoding

#### 2. **Rendering Pipeline**
```cpp
// Main rendering pass (visible to user)
framebuffer_main.Bind();
point_cloud.Render(NORMAL_MODE);

// ID picking pass (off-screen)
framebuffer_id.Bind();
glClearColor(0, 0, 0, 0);  // Background = ID 0
point_cloud.Render(ID_MODE);
```

#### 3. **Point Selection Process**
```cpp
// Mouse click event
glm::ivec2 mouse_pos = GetMousePosition();

// Read pixel from ID buffer
framebuffer_id.Bind();
GLubyte pixel[3];
glReadPixels(mouse_pos.x, mouse_pos.y, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, pixel);

// Decode point index
uint32_t id = (pixel[0] << 16) | (pixel[1] << 8) | pixel[2];
size_t point_index = (id > 0) ? id - 1 : INVALID_POINT;
```

#### 4. **Shader Implementation**
```glsl
// ID picking vertex shader
#version 330 core
layout (location = 0) in vec3 position;
uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

void main() {
    gl_Position = projection * view * model * vec4(position, 1.0);
    gl_PointSize = point_size;
}

// ID picking fragment shader  
#version 330 core
uniform uint point_id;
out vec3 fragColor;

void main() {
    // Circular point shape (discard pixels outside circle)
    vec2 coord = gl_PointCoord - vec2(0.5);
    if (dot(coord, coord) > 0.25) discard;
    
    // Encode point ID in RGB
    fragColor = vec3(
        float((point_id >> 16) & 0xFF) / 255.0,
        float((point_id >> 8) & 0xFF) / 255.0, 
        float(point_id & 0xFF) / 255.0
    );
}
```

### Advantages Over Ray-Casting

**Pixel-Perfect Accuracy**:
- Selected point exactly matches what user sees on screen
- Automatic handling of overlapping points via depth testing
- No ambiguity about which point should be selected

**Occlusion Handling**:
- GPU depth buffer automatically handles point visibility
- Only frontmost points can be selected
- Perfect handling of dense point clouds with layered structure

**Camera Independence**:
- Works regardless of camera position, orientation, or projection type
- No sensitivity to camera distance from point cloud
- Consistent behavior across all viewing angles

**Performance**:
- Leverages GPU parallel processing for ID rendering
- O(1) selection time regardless of point cloud size
- Minimal CPU overhead for pixel readback

**Scalability**:
- Handles point clouds from hundreds to millions of points
- Memory usage scales with framebuffer size, not point count
- Easy to extend with area/brush selection tools

### Implementation Strategy

#### Phase 1: Core Infrastructure
1. **Extend PointCloud Renderable**:
   - Add ID rendering mode alongside normal rendering
   - Implement ID encoding in shaders
   - Support for switching between render modes

2. **GlSceneManager Integration**:
   - Add dedicated ID framebuffer with same dimensions as main buffer
   - Render ID pass before/after main rendering pass
   - Provide mouse-to-point-index API

3. **InteractiveSceneManager Integration**:
   - Replace ray-casting with framebuffer pixel reads
   - Maintain existing selection callback interface
   - Preserve current modifier key behavior (Ctrl+Click, etc.)

#### Phase 2: Enhanced Features
1. **Radius Picking**:
   - Read 3x3 or 5x5 pixel block around mouse position
   - Return closest point among candidates
   - Provides tolerance without distance calculations

2. **Area Selection Tools**:
   - Rectangle selection: render selection region, read all pixels
   - Lasso selection: render arbitrary polygon, read enclosed pixels
   - Brush selection: continuous area picking during mouse drag

3. **Performance Optimization**:
   - Lazy ID buffer updates (only when needed)
   - Framebuffer size optimization (can be lower resolution than main buffer)
   - Memory pooling for pixel readback operations

#### Phase 3: Advanced Features
1. **Hybrid CPU/GPU Approach**:
   - GPU picking for single-point precision
   - CPU k-d tree for region queries and algorithms
   - Best of both worlds: precision + advanced queries

2. **Large Point Cloud Support**:
   - Tiling for >16M point clouds
   - Level-of-detail integration
   - Streaming point cloud support

### Integration with Existing Architecture

#### Minimal Changes Required
The GPU ID-buffer approach integrates cleanly with QuickViz's existing architecture:

**PointCloud Class**:
- Add `SetRenderMode(RenderMode::kIdBuffer)` 
- Existing shader infrastructure supports mode switching
- Layer system remains unchanged

**GlSceneManager**:
- Add ID framebuffer as member variable
- Existing framebuffer utilities support additional buffers
- No changes to object management or rendering pipeline

**InteractiveSceneManager**:
- Replace `PickPoint()` call with framebuffer read
- Maintain existing callback and selection mode interfaces
- No changes to user-facing selection API

#### Backward Compatibility
- Keep existing ray-casting code as fallback option
- CPU k-d tree remains for region/analytical queries
- Existing selection visualization system unchanged

### Technical Considerations

#### Memory Usage
- ID framebuffer: `width × height × 3 bytes` (RGB)
- For 1920×1080: ~6MB additional GPU memory
- Negligible compared to point cloud vertex data

#### Precision Limitations
- 24-bit encoding supports ~16.7M unique points
- For larger clouds: 32-bit RGBA encoding or tiling approach
- Most practical applications well within 16M point limit

#### Performance Impact
- Additional render pass: ~1-2ms for typical point clouds
- Pixel readback: <1ms for single pixel or small regions
- Overall selection latency: <5ms (vs 10-50ms for ray-casting)

#### Error Handling
- Background color (ID 0) indicates no point selected
- Invalid IDs handled gracefully with bounds checking
- Framebuffer read failures fall back to ray-casting

## Conclusion

GPU ID-buffer picking provides a robust, industry-standard solution that addresses all limitations of the current ray-casting approach. The implementation integrates cleanly with QuickViz's existing architecture while providing pixel-perfect selection accuracy and excellent performance scaling.

This approach will provide users with intuitive, reliable point selection that matches professional visualization tools, while maintaining the flexibility to extend with advanced selection features in the future.

## References

- [OpenGL Red Book: Color Index Mode and Picking](https://www.opengl.org/archives/resources/features/KilgardTechniques/oglpitfall/)
- [ParaView Selection Implementation](https://www.paraview.org/Wiki/Selection_Implementation_in_VTK_and_ParaView)
- [Real-Time Rendering: Selection and Picking Techniques](https://www.realtimerendering.com/)