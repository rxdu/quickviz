# Sphere Migration Example

*Example migration of Sphere class to use GeometricPrimitive base class*  
*Date: August 26, 2025*

## Migration Strategy

This document shows how to migrate the existing Sphere class to inherit from GeometricPrimitive while maintaining backward compatibility.

## Phase 1: Non-Breaking Migration

### Updated Sphere Header

```cpp
// sphere.hpp - Updated to inherit from GeometricPrimitive
#include "scene/renderable/geometric_primitive.hpp"

namespace quickviz {

/**
 * @brief Renderable 3D sphere for waypoints and detection zones
 * Now inherits unified GeometricPrimitive interface
 */
class Sphere : public GeometricPrimitive {
public:
    // =================================================================
    // Backward Compatible Constructors and Methods
    // =================================================================
    
    Sphere();
    Sphere(const glm::vec3& center, float radius);
    ~Sphere();

    // Keep existing specific methods for backward compatibility
    void SetCenter(const glm::vec3& center);
    void SetRadius(float radius);
    glm::vec3 GetCenter() const { return center_; }
    float GetRadius() const { return radius_; }
    
    // Keep existing appearance methods (now forward to base class)
    void SetColor(const glm::vec3& color) override { GeometricPrimitive::SetColor(color); }
    void SetWireframeColor(const glm::vec3& color) override { GeometricPrimitive::SetWireframeColor(color); }
    void SetOpacity(float opacity) override { GeometricPrimitive::SetOpacity(opacity); }
    
    // Existing render mode support (now uses unified enum)
    void SetRenderMode(RenderMode mode) override;
    
    // Existing quality settings
    void SetResolution(int latitude_segments, int longitude_segments);
    
    // =================================================================
    // GeometricPrimitive Interface Implementation
    // =================================================================
    
    // Transform interface
    void SetTransform(const glm::mat4& transform) override;
    glm::mat4 GetTransform() const override;
    
    // Geometry calculations
    float GetVolume() const override;
    float GetSurfaceArea() const override;
    glm::vec3 GetCentroid() const override;
    std::pair<glm::vec3, glm::vec3> GetBoundingBox() const override;
    
    // OpenGL resource management (unchanged)
    void AllocateGpuResources() override;
    void ReleaseGpuResources() noexcept override;
    bool IsGpuResourcesAllocated() const noexcept override { return vao_ != 0; }

protected:
    // =================================================================
    // Template Method Implementation
    // =================================================================
    
    void PrepareShaders(const glm::mat4& mvp_matrix, const glm::mat4& model_matrix) override;
    void RenderSolid() override;
    void RenderWireframe() override;
    void RenderPoints() override;

private:
    // Sphere-specific geometry
    glm::vec3 center_ = glm::vec3(0.0f);
    float radius_ = 1.0f;
    int latitude_segments_ = 20;
    int longitude_segments_ = 20;
    
    // OpenGL resources (unchanged)
    uint32_t vao_ = 0;
    uint32_t position_vbo_ = 0;
    uint32_t normal_vbo_ = 0;
    uint32_t ebo_ = 0;
    
    // Geometry data
    std::vector<glm::vec3> vertices_;
    std::vector<glm::vec3> normals_;
    std::vector<uint32_t> indices_;
    
    // Internal methods (unchanged)
    void GenerateSphereGeometry();
    void UpdateGpuBuffers();
    void UpdateTransformFromCenterRadius();
};

} // namespace quickviz
```

### Key Implementation Methods

```cpp
// sphere.cpp - Key methods showing the migration approach

void Sphere::SetTransform(const glm::mat4& transform) {
    // Extract center and radius from transform matrix if possible
    // For now, use the full transform matrix
    transform_ = transform;
    MarkForUpdate();
}

glm::mat4 Sphere::GetTransform() const {
    // Create transform matrix from center and radius
    glm::mat4 transform = glm::mat4(1.0f);
    transform = glm::translate(transform, center_);
    transform = glm::scale(transform, glm::vec3(radius_));
    return transform;
}

float Sphere::GetVolume() const {
    return (4.0f / 3.0f) * M_PI * radius_ * radius_ * radius_;
}

float Sphere::GetSurfaceArea() const {
    return 4.0f * M_PI * radius_ * radius_;
}

glm::vec3 Sphere::GetCentroid() const {
    return center_;
}

std::pair<glm::vec3, glm::vec3> Sphere::GetBoundingBox() const {
    glm::vec3 extent(radius_);
    return {center_ - extent, center_ + extent};
}

void Sphere::PrepareShaders(const glm::mat4& mvp_matrix, const glm::mat4& model_matrix) {
    // Use the appropriate shared shader based on render mode
    ShaderProgram* shader = nullptr;
    
    switch (render_mode_) {
        case RenderMode::kSolid:
            shader = solid_shader_.get();
            break;
        case RenderMode::kTransparent:
            shader = transparent_shader_.get();
            break;
        case RenderMode::kWireframe:
        case RenderMode::kOutline:
            shader = wireframe_shader_.get();
            break;
        case RenderMode::kPoints:
            shader = point_shader_.get();
            break;
    }
    
    if (!shader) return;
    
    shader->Use();
    shader->SetMatrix4("mvp", mvp_matrix);
    shader->SetMatrix4("model", model_matrix);
    
    // Setup material properties
    if (render_mode_ == RenderMode::kSolid || render_mode_ == RenderMode::kTransparent) {
        shader->SetVector3f("diffuse_color", material_.diffuse_color);
        shader->SetFloat("opacity", material_.opacity);
        shader->SetBool("use_lighting", material_.use_lighting);
        
        if (material_.use_lighting) {
            glm::mat4 normal_matrix = glm::transpose(glm::inverse(model_matrix));
            shader->SetMatrix4("normal_matrix", normal_matrix);
            shader->SetVector3f("light_pos", glm::vec3(10.0f, 10.0f, 10.0f));
            shader->SetVector3f("light_color", glm::vec3(1.0f, 1.0f, 1.0f));
            shader->SetFloat("ambient_factor", material_.ambient_factor);
            shader->SetFloat("diffuse_factor", material_.diffuse_factor);
            shader->SetFloat("specular_factor", material_.specular_factor);
        }
    } else if (render_mode_ == RenderMode::kWireframe || render_mode_ == RenderMode::kOutline) {
        shader->SetVector3f("wireframe_color", material_.wireframe_color);
        shader->SetFloat("opacity", material_.opacity);
    } else if (render_mode_ == RenderMode::kPoints) {
        shader->SetVector3f("diffuse_color", material_.diffuse_color);
        shader->SetFloat("opacity", material_.opacity);
        shader->SetFloat("point_size", point_size_);
    }
}

void Sphere::RenderSolid() {
    if (vao_ == 0) return;
    
    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void Sphere::RenderWireframe() {
    if (vao_ == 0) return;
    
    glBindVertexArray(vao_);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_INT, 0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBindVertexArray(0);
}

void Sphere::RenderPoints() {
    if (vao_ == 0) return;
    
    glBindVertexArray(vao_);
    glDrawArrays(GL_POINTS, 0, vertices_.size());
    glBindVertexArray(0);
}
```

## Usage Examples

### Before Migration (Still Works)
```cpp
auto sphere = std::make_unique<Sphere>(glm::vec3(0, 0, 0), 1.5f);
sphere->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
sphere->SetRenderMode(Sphere::RenderMode::kTransparent);
sphere->SetOpacity(0.7f);
```

### After Migration (New Unified API)
```cpp
auto sphere = std::make_unique<Sphere>(glm::vec3(0, 0, 0), 1.5f);

// Use unified material system
GeometricPrimitive::Material material;
material.diffuse_color = glm::vec3(1.0f, 0.0f, 0.0f);
material.opacity = 0.7f;
material.use_lighting = true;
sphere->SetMaterial(material);

// Use unified render modes
sphere->SetRenderMode(GeometricPrimitive::RenderMode::kTransparent);

// New capabilities from base class
sphere->SetHighlighted(true);  // Selection highlighting
auto bounds = sphere->GetBoundingBox();  // Picking support
float volume = sphere->GetVolume();  // Geometry calculations
```

### Polymorphic Usage (New Capability)
```cpp
std::vector<std::unique_ptr<GeometricPrimitive>> primitives;
primitives.push_back(std::make_unique<Sphere>(glm::vec3(0, 0, 0), 1.0f));
primitives.push_back(std::make_unique<Cylinder>(glm::vec3(2, 0, 0), glm::vec3(2, 3, 0), 0.5f));
primitives.push_back(std::make_unique<BoundingBox>(glm::vec3(-2, 0, 0), glm::vec3(1, 1, 1)));

// Unified interface for all primitives
for (auto& primitive : primitives) {
    primitive->SetColor(glm::vec3(0.8f, 0.2f, 0.2f));
    primitive->SetRenderMode(GeometricPrimitive::RenderMode::kWireframe);
    primitive->SetHighlighted(true);
    
    // Geometry calculations work for all types
    std::cout << "Volume: " << primitive->GetVolume() << std::endl;
    std::cout << "Surface Area: " << primitive->GetSurfaceArea() << std::endl;
}
```

## Benefits Achieved

### 1. **API Consistency**
- All primitives now have identical interfaces
- Consistent material and rendering systems
- Unified selection and highlighting

### 2. **Code Reuse**
- Shared shader programs across all primitives
- Common OpenGL state management
- Unified rendering pipeline

### 3. **Polymorphic Capabilities**
- Store different primitives in same containers
- Apply operations uniformly across primitive types
- Easier integration with scene management

### 4. **Enhanced Features**
- Built-in selection support for all primitives
- Consistent geometry calculation methods
- Extensible material system for future PBR support

### 5. **Maintained Compatibility**
- Existing sphere-specific methods still work
- Gradual migration path available
- No breaking changes for existing code

## Migration Timeline

### Phase 1 (Week 1): Foundation
- [x] Create GeometricPrimitive base class
- [x] Implement shared shader system
- [x] Design migration strategy

### Phase 2 (Week 2): Sphere Migration
- [ ] Update Sphere to inherit from GeometricPrimitive
- [ ] Implement template method hooks
- [ ] Test backward compatibility
- [ ] Update Sphere tests

### Phase 3 (Week 3): Additional Primitives
- [ ] Migrate Cylinder class
- [ ] Migrate BoundingBox class
- [ ] Create unified primitive tests

### Phase 4 (Week 4): Polish & Documentation
- [ ] Update API documentation
- [ ] Create usage examples
- [ ] Performance validation
- [ ] Deprecate old inconsistent methods

This migration provides significant architectural improvements while maintaining full backward compatibility, making it a safe and valuable enhancement to the codebase.