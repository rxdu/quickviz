/**
 * @file geometric_primitive.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-26
 * @brief Implementation of unified GeometricPrimitive base class
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/renderable/geometric_primitive.hpp"

#include <glad/glad.h>
#include <iostream>
#include <stdexcept>

#include "gldraw/shader.hpp"

namespace quickviz {

// =================================================================
// Static Member Initialization
// =================================================================

std::unique_ptr<ShaderProgram> GeometricPrimitive::solid_shader_ = nullptr;
std::unique_ptr<ShaderProgram> GeometricPrimitive::wireframe_shader_ = nullptr;
std::unique_ptr<ShaderProgram> GeometricPrimitive::transparent_shader_ = nullptr;
std::unique_ptr<ShaderProgram> GeometricPrimitive::point_shader_ = nullptr;
bool GeometricPrimitive::shaders_initialized_ = false;

// =================================================================
// Constructor and Destructor
// =================================================================

GeometricPrimitive::GeometricPrimitive() {
    // Initialize shared shaders if not already done
    if (!shaders_initialized_) {
        InitializeShaders();
    }
    
    // Initialize with default material
    material_ = Material{};
    original_material_ = material_;
}

GeometricPrimitive::~GeometricPrimitive() {
    // Note: We don't call CleanupShaders() here because the shaders are static
    // and shared among all instances. They should only be cleaned up when
    // explicitly requested or at program termination.
}

// =================================================================
// Material and Appearance Interface
// =================================================================

void GeometricPrimitive::SetMaterial(const Material& material) {
    if (!is_highlighted_) {
        original_material_ = material;
    }
    material_ = material;
    MarkForUpdate();
}

void GeometricPrimitive::SetColor(const glm::vec3& color) {
    material_.diffuse_color = color;
    if (!is_highlighted_) {
        original_material_.diffuse_color = color;
    }
    MarkForUpdate();
}

void GeometricPrimitive::SetWireframeColor(const glm::vec3& color) {
    material_.wireframe_color = color;
    if (!is_highlighted_) {
        original_material_.wireframe_color = color;
    }
    MarkForUpdate();
}

void GeometricPrimitive::SetOpacity(float opacity) {
    material_.opacity = glm::clamp(opacity, 0.0f, 1.0f);
    if (!is_highlighted_) {
        original_material_.opacity = material_.opacity;
    }
    MarkForUpdate();
}

void GeometricPrimitive::SetRenderMode(RenderMode mode) {
    render_mode_ = mode;
    MarkForUpdate();
}

void GeometricPrimitive::SetBlendMode(BlendMode mode) {
    blend_mode_ = mode;
    MarkForUpdate();
}

// =================================================================
// Rendering Quality Interface
// =================================================================

void GeometricPrimitive::SetWireframeWidth(float width) {
    wireframe_width_ = std::max(width, 0.1f);  // Minimum line width
    MarkForUpdate();
}

void GeometricPrimitive::SetPointSize(float size) {
    point_size_ = std::max(size, 1.0f);  // Minimum point size
    MarkForUpdate();
}

// =================================================================
// Selection Interface
// =================================================================

void GeometricPrimitive::SetHighlighted(bool highlighted) {
    if (is_highlighted_ == highlighted) return;  // No change needed
    
    is_highlighted_ = highlighted;
    
    if (highlighted) {
        // Save original material and apply highlight
        original_material_ = material_;
        material_.diffuse_color = material_.highlight_color;
        material_.wireframe_color = material_.highlight_color;
        // Increase opacity slightly for better visibility
        material_.opacity = std::min(1.0f, original_material_.opacity + 0.2f);
    } else {
        // Restore original material
        material_ = original_material_;
    }
    
    MarkForUpdate();
}

// =================================================================
// Main Rendering Method (Template Method Pattern)
// =================================================================

void GeometricPrimitive::OnDraw(const glm::mat4& projection, const glm::mat4& view,
                               const glm::mat4& coord_transform) {
    try {
        // Calculate transformation matrices
        glm::mat4 model_matrix = GetTransform();
        glm::mat4 mvp_matrix = projection * view * coord_transform * model_matrix;
        
        // Handle ID rendering mode for GPU selection
        if (id_render_mode_) {
            RenderIdBuffer(mvp_matrix);
            return;
        }
        
        // Setup OpenGL state for this render mode
        SetupRenderState();
        
        // Prepare shaders and uniforms (subclass responsibility)
        PrepareShaders(mvp_matrix, model_matrix);
        
        // Call appropriate render method based on mode
        switch (render_mode_) {
            case RenderMode::kSolid:
            case RenderMode::kTransparent:
                RenderSolid();
                break;
                
            case RenderMode::kWireframe:
            case RenderMode::kOutline:
                RenderWireframe();
                break;
                
            case RenderMode::kPoints:
                RenderPoints();
                break;
        }
        
        // Restore OpenGL state
        RestoreRenderState();
        
    } catch (const std::exception& e) {
        std::cerr << "Error rendering geometric primitive: " << e.what() << std::endl;
        // Continue with next object rather than crashing
    }
}

// =================================================================
// OpenGL State Management
// =================================================================

void GeometricPrimitive::SetupRenderState() {
    // Handle transparency and blending
    bool is_transparent = (render_mode_ == RenderMode::kTransparent || material_.opacity < 1.0f);
    
    if (is_transparent) {
        glEnable(GL_BLEND);
        
        // For transparent render mode, default to alpha blending if no specific blend mode set
        BlendMode effective_blend_mode = blend_mode_;
        if (render_mode_ == RenderMode::kTransparent && blend_mode_ == BlendMode::kOpaque) {
            effective_blend_mode = BlendMode::kAlpha;  // Auto-enable alpha blending for transparency
        }
        
        switch (effective_blend_mode) {
            case BlendMode::kAlpha:
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                break;
            case BlendMode::kAdditive:
                glBlendFunc(GL_SRC_ALPHA, GL_ONE);
                break;
            case BlendMode::kMultiply:
                glBlendFunc(GL_DST_COLOR, GL_ZERO);
                break;
            case BlendMode::kOpaque:
            default:
                glDisable(GL_BLEND);
                break;
        }
        
        // CRITICAL: Disable depth writing for transparency but keep depth testing
        // This prevents transparent objects from blocking objects behind them
        glDepthMask(GL_FALSE);
    } else {
        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);  // Enable depth writing for opaque objects
    }
    
    // Setup depth testing
    glEnable(GL_DEPTH_TEST);
    
    // Setup proper depth testing - no special bias, respect actual geometry positions
    glDepthFunc(GL_LESS);
    
    // Only use polygon offset for pure wireframe objects to avoid z-fighting with their own surfaces
    if (render_mode_ == RenderMode::kWireframe || render_mode_ == RenderMode::kOutline) {
        glEnable(GL_POLYGON_OFFSET_LINE);
        glPolygonOffset(-0.1f, -0.1f);  // Minimal offset just to avoid z-fighting
        glDepthMask(GL_TRUE);  // Wireframes write to depth buffer
    } else {
        glDisable(GL_POLYGON_OFFSET_LINE);
    }
    
    // Setup wireframe width if needed
    if (render_mode_ == RenderMode::kWireframe || render_mode_ == RenderMode::kOutline) {
        glLineWidth(wireframe_width_);
    }
    
    // Setup point size if needed
    if (render_mode_ == RenderMode::kPoints) {
        glPointSize(point_size_);
    }
    
    // Handle backface culling based on transparency
    if (is_transparent) {
        glDisable(GL_CULL_FACE);  // Show both sides for transparent objects
    } else {
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
    }
}

void GeometricPrimitive::RestoreRenderState() {
    // Reset line width and point size to defaults
    glLineWidth(1.0f);
    glPointSize(1.0f);
    
    // Restore standard culling
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    
    // Restore depth writing for subsequent opaque objects
    glDepthMask(GL_TRUE);
    
    // Disable polygon offset for wireframes
    glDisable(GL_POLYGON_OFFSET_LINE);
    
    // Restore standard depth function
    glDepthFunc(GL_LESS);
    
    // Keep depth testing and blending as-is for other objects
}

// =================================================================
// Shared Shader Management
// =================================================================

void GeometricPrimitive::InitializeShaders() {
    if (shaders_initialized_) return;
    
    try {
        // Solid rendering shader with Phong lighting
        const char* solid_vertex_source = R"(
            #version 330 core
            layout (location = 0) in vec3 aPos;
            layout (location = 1) in vec3 aNormal;
            
            uniform mat4 mvp;
            uniform mat4 model;
            uniform mat4 normal_matrix;
            
            out vec3 FragPos;
            out vec3 Normal;
            
            void main() {
                FragPos = vec3(model * vec4(aPos, 1.0));
                Normal = mat3(normal_matrix) * aNormal;
                gl_Position = mvp * vec4(aPos, 1.0);
            }
        )";
        
        const char* solid_fragment_source = R"(
            #version 330 core
            in vec3 FragPos;
            in vec3 Normal;
            
            uniform vec3 diffuse_color;
            uniform float opacity;
            uniform vec3 light_pos;
            uniform vec3 light_color;
            uniform vec3 view_pos;
            uniform float ambient_factor;
            uniform float diffuse_factor;
            uniform float specular_factor;
            uniform bool use_lighting;
            
            out vec4 FragColor;
            
            void main() {
                vec3 color = diffuse_color;
                
                if (use_lighting) {
                    // Ambient
                    vec3 ambient = ambient_factor * light_color;
                    
                    // Diffuse
                    vec3 norm = normalize(Normal);
                    vec3 light_dir = normalize(light_pos - FragPos);
                    float diff = max(dot(norm, light_dir), 0.0);
                    vec3 diffuse = diffuse_factor * diff * light_color;
                    
                    // Specular (Phong)
                    vec3 view_dir = normalize(view_pos - FragPos);
                    vec3 reflect_dir = reflect(-light_dir, norm);
                    float spec = pow(max(dot(view_dir, reflect_dir), 0.0), 32.0);
                    vec3 specular = specular_factor * spec * light_color;
                    
                    color = color * (ambient + diffuse) + specular;
                }
                
                FragColor = vec4(color, opacity);
            }
        )";
        
        solid_shader_ = std::make_unique<ShaderProgram>();
        // Use the existing shader compilation pattern from this codebase
        Shader solid_vs(solid_vertex_source, Shader::Type::kVertex);
        Shader solid_fs(solid_fragment_source, Shader::Type::kFragment);
        if (!solid_vs.Compile() || !solid_fs.Compile()) {
            throw std::runtime_error("Solid shader compilation failed");
        }
        solid_shader_->AttachShader(solid_vs);
        solid_shader_->AttachShader(solid_fs);
        if (!solid_shader_->LinkProgram()) {
            throw std::runtime_error("Solid shader linking failed");
        }
        
        // Wireframe shader (simpler, no lighting)
        const char* wireframe_fragment_source = R"(
            #version 330 core
            uniform vec3 wireframe_color;
            uniform float opacity;
            out vec4 FragColor;
            
            void main() {
                FragColor = vec4(wireframe_color, opacity);
            }
        )";
        
        wireframe_shader_ = std::make_unique<ShaderProgram>();
        Shader wireframe_vs(solid_vertex_source, Shader::Type::kVertex);
        Shader wireframe_fs(wireframe_fragment_source, Shader::Type::kFragment);
        if (!wireframe_vs.Compile() || !wireframe_fs.Compile()) {
            throw std::runtime_error("Wireframe shader compilation failed");
        }
        wireframe_shader_->AttachShader(wireframe_vs);
        wireframe_shader_->AttachShader(wireframe_fs);
        if (!wireframe_shader_->LinkProgram()) {
            throw std::runtime_error("Wireframe shader linking failed");
        }
        
        // Transparent shader (same as solid but with alpha blending support)
        transparent_shader_ = std::make_unique<ShaderProgram>();
        Shader trans_vs(solid_vertex_source, Shader::Type::kVertex);
        Shader trans_fs(solid_fragment_source, Shader::Type::kFragment);
        if (!trans_vs.Compile() || !trans_fs.Compile()) {
            throw std::runtime_error("Transparent shader compilation failed");
        }
        transparent_shader_->AttachShader(trans_vs);
        transparent_shader_->AttachShader(trans_fs);
        if (!transparent_shader_->LinkProgram()) {
            throw std::runtime_error("Transparent shader linking failed");
        }
        
        // Point shader
        const char* point_vertex_source = R"(
            #version 330 core
            layout (location = 0) in vec3 aPos;
            
            uniform mat4 mvp;
            uniform float point_size;
            
            void main() {
                gl_Position = mvp * vec4(aPos, 1.0);
                gl_PointSize = point_size;
            }
        )";
        
        const char* point_fragment_source = R"(
            #version 330 core
            uniform vec3 diffuse_color;
            uniform float opacity;
            out vec4 FragColor;
            
            void main() {
                // Create circular points
                vec2 coord = gl_PointCoord - vec2(0.5);
                if (dot(coord, coord) > 0.25) discard;
                FragColor = vec4(diffuse_color, opacity);
            }
        )";
        
        point_shader_ = std::make_unique<ShaderProgram>();
        Shader point_vs(point_vertex_source, Shader::Type::kVertex);
        Shader point_fs(point_fragment_source, Shader::Type::kFragment);
        if (!point_vs.Compile() || !point_fs.Compile()) {
            throw std::runtime_error("Point shader compilation failed");
        }
        point_shader_->AttachShader(point_vs);
        point_shader_->AttachShader(point_fs);
        if (!point_shader_->LinkProgram()) {
            throw std::runtime_error("Point shader linking failed");
        }
        
        shaders_initialized_ = true;
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize geometric primitive shaders: " << e.what() << std::endl;
        throw;
    }
}

void GeometricPrimitive::CleanupShaders() {
    solid_shader_.reset();
    wireframe_shader_.reset(); 
    transparent_shader_.reset();
    point_shader_.reset();
    shaders_initialized_ = false;
}

// =================================================================
// Utility Functions
// =================================================================

const char* RenderModeToString(GeometricPrimitive::RenderMode mode) {
    switch (mode) {
        case GeometricPrimitive::RenderMode::kSolid: return "Solid";
        case GeometricPrimitive::RenderMode::kWireframe: return "Wireframe";
        case GeometricPrimitive::RenderMode::kTransparent: return "Transparent";
        case GeometricPrimitive::RenderMode::kPoints: return "Points";
        case GeometricPrimitive::RenderMode::kOutline: return "Outline";
        default: return "Unknown";
    }
}

// =================================================================
// ID Buffer Rendering (Default Implementation)
// =================================================================

void GeometricPrimitive::RenderIdBuffer(const glm::mat4& mvp_matrix) {
    // Default implementation: create simple ID shader and render solid geometry
    // Subclasses can override for more specialized ID rendering
    
    // Create static ID shader (shared across all instances)
    static std::unique_ptr<ShaderProgram> id_shader = nullptr;
    static bool shader_initialized = false;
    
    if (!shader_initialized) {
        try {
            // Simple ID vertex shader
            const char* id_vertex_shader = R"(
                #version 330 core
                layout (location = 0) in vec3 aPos;
                
                uniform mat4 uMVP;
                
                void main() {
                    gl_Position = uMVP * vec4(aPos, 1.0);
                }
            )";
            
            // Simple ID fragment shader
            const char* id_fragment_shader = R"(
                #version 330 core
                out vec4 FragColor;
                
                uniform vec3 uIdColor;
                
                void main() {
                    FragColor = vec4(uIdColor, 1.0);
                }
            )";
            
            id_shader = std::make_unique<ShaderProgram>();
            Shader vs(id_vertex_shader, Shader::Type::kVertex);
            Shader fs(id_fragment_shader, Shader::Type::kFragment);
            
            if (!vs.Compile() || !fs.Compile()) {
                std::cerr << "GeometricPrimitive: ID shader compilation failed" << std::endl;
                shader_initialized = true; // Prevent retry
                return;
            }
            
            id_shader->AttachShader(vs);
            id_shader->AttachShader(fs);
            
            if (!id_shader->LinkProgram()) {
                std::cerr << "GeometricPrimitive: ID shader linking failed" << std::endl;
                shader_initialized = true; // Prevent retry
                return;
            }
            
            shader_initialized = true;
        } catch (const std::exception& e) {
            std::cerr << "GeometricPrimitive: ID shader initialization failed: " << e.what() << std::endl;
            shader_initialized = true; // Prevent retry
            return;
        }
    }
    
    if (!id_shader) {
        return; // Shader creation failed
    }
    
    // Use the ID shader and render solid geometry
    id_shader->Use();
    id_shader->SetUniform("uMVP", mvp_matrix);
    id_shader->SetUniform("uIdColor", id_color_);
    
    // Call RenderSolid() to draw the geometry with ID color
    RenderSolid();
}

const char* BlendModeToString(GeometricPrimitive::BlendMode mode) {
    switch (mode) {
        case GeometricPrimitive::BlendMode::kOpaque: return "Opaque";
        case GeometricPrimitive::BlendMode::kAlpha: return "Alpha";
        case GeometricPrimitive::BlendMode::kAdditive: return "Additive";
        case GeometricPrimitive::BlendMode::kMultiply: return "Multiply";
        default: return "Unknown";
    }
}

} // namespace quickviz