/**
 * @file geometric_primitive.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-26
 * @brief Unified base class for all 3D geometric primitives
 * 
 * This class provides a consistent interface across all geometric primitives
 * (Sphere, Cylinder, BoundingBox, etc.) while preserving specialized functionality.
 * It implements the Template Method pattern for efficient rendering with shared
 * common code and unified material/selection systems.
 * 
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_GEOMETRIC_PRIMITIVE_HPP
#define QUICKVIZ_GEOMETRIC_PRIMITIVE_HPP

#include <glm/glm.hpp>
#include <memory>

#include "gldraw/interface/opengl_object.hpp"
#include "gldraw/shader_program.hpp"

namespace quickviz {

/**
 * @brief Base class for all 3D geometric primitives
 * 
 * Provides unified interface for appearance, rendering, and interaction
 * while allowing specialized functionality in derived classes.
 * 
 * Key Features:
 * - Consistent material and appearance system
 * - Unified selection and highlighting support  
 * - Template Method pattern for efficient rendering
 * - Shared shader resources across instances
 * - Extensible material system for future PBR support
 */
class GeometricPrimitive : public OpenGlObject {
public:
    /**
     * @brief Standardized render modes across all primitives
     */
    enum class RenderMode {
        kSolid,        ///< Filled surface with lighting
        kWireframe,    ///< Edges/wireframe only
        kTransparent,  ///< Transparent surface with alpha blending
        kPoints,       ///< Point cloud representation
        kOutline       ///< Silhouette/outline only
    };

    /**
     * @brief Blending modes for transparency and effects
     */
    enum class BlendMode {
        kOpaque,       ///< No blending (default)
        kAlpha,        ///< Standard alpha blending
        kAdditive,     ///< Additive blending for glow effects
        kMultiply      ///< Multiplicative blending
    };

    /**
     * @brief Unified material system for all primitives
     * 
     * Designed to be extensible for future PBR (Physically Based Rendering) support
     * while maintaining backward compatibility with simple color-based materials.
     */
    struct Material {
        glm::vec3 diffuse_color = glm::vec3(0.7f, 0.7f, 0.9f);    ///< Base surface color
        glm::vec3 wireframe_color = glm::vec3(0.0f, 0.0f, 0.0f);  ///< Wireframe/edge color
        glm::vec3 highlight_color = glm::vec3(1.0f, 1.0f, 0.0f);  ///< Selection highlight color
        
        float opacity = 1.0f;          ///< Alpha value (0.0 = transparent, 1.0 = opaque)
        float metallic = 0.0f;         ///< Metallic factor for PBR (0.0 = dielectric, 1.0 = metallic)  
        float roughness = 0.5f;        ///< Surface roughness for PBR (0.0 = mirror, 1.0 = rough)
        float ambient_factor = 0.1f;   ///< Ambient light contribution
        float diffuse_factor = 0.7f;   ///< Diffuse light contribution
        float specular_factor = 0.2f;  ///< Specular light contribution
        
        bool use_lighting = true;      ///< Enable/disable lighting calculations
        bool cast_shadows = true;      ///< Enable shadow casting (future feature)
        bool receive_shadows = true;   ///< Enable shadow receiving (future feature)
    };

    /**
     * @brief Virtual destructor for proper cleanup
     */
    virtual ~GeometricPrimitive();

    // =================================================================
    // Transform and Positioning Interface
    // =================================================================

    /**
     * @brief Set the complete transform matrix for the primitive
     * @param transform 4x4 transformation matrix (translation, rotation, scale)
     */
    virtual void SetTransform(const glm::mat4& transform) = 0;

    /**
     * @brief Get the current transform matrix
     * @return Current transformation matrix
     */
    virtual glm::mat4 GetTransform() const = 0;

    // =================================================================
    // Material and Appearance Interface
    // =================================================================

    /**
     * @brief Set the complete material properties
     * @param material Material struct with all appearance properties
     */
    virtual void SetMaterial(const Material& material);

    /**
     * @brief Get current material properties
     * @return Reference to current material
     */
    virtual const Material& GetMaterial() const { return material_; }

    /**
     * @brief Set the main surface color (convenience method)
     * @param color RGB color values (0.0-1.0)
     */
    virtual void SetColor(const glm::vec3& color);

    /**
     * @brief Set the wireframe/edge color (convenience method)  
     * @param color RGB color values (0.0-1.0)
     */
    virtual void SetWireframeColor(const glm::vec3& color);

    /**
     * @brief Set the transparency level
     * @param opacity Alpha value (0.0 = transparent, 1.0 = opaque)
     */
    virtual void SetOpacity(float opacity);

    /**
     * @brief Set the rendering mode
     * @param mode How the primitive should be rendered
     */
    virtual void SetRenderMode(RenderMode mode);

    /**
     * @brief Set the blending mode for transparency effects
     * @param mode Blending equation to use
     */
    virtual void SetBlendMode(BlendMode mode);

    // =================================================================
    // Rendering Quality Interface
    // =================================================================

    /**
     * @brief Set wireframe line width
     * @param width Line width in pixels
     */
    virtual void SetWireframeWidth(float width);

    /**
     * @brief Set point size for point rendering mode
     * @param size Point size in pixels
     */
    virtual void SetPointSize(float size);

    // =================================================================
    // Selection Interface (OpenGlObject Implementation)
    // =================================================================

    /**
     * @brief Set selection highlight state
     * @param highlighted True to show selection highlight
     */
    void SetHighlighted(bool highlighted) override;

    /**
     * @brief Check if primitive supports selection
     * @return Always true - all geometric primitives support selection
     */
    bool SupportsSelection() const override { return true; }

    /**
     * @brief Get axis-aligned bounding box for picking
     * @return Pair of {min_bounds, max_bounds} in world space
     */
    std::pair<glm::vec3, glm::vec3> GetBoundingBox() const override = 0;
    
    // =================================================================
    // GPU ID-Buffer Selection Support  
    // =================================================================
    
    /**
     * @brief Enable/disable ID rendering mode for GPU selection
     * @param enabled True to render with solid ID color, false for normal rendering
     */
    void SetIdRenderMode(bool enabled) override { id_render_mode_ = enabled; }
    
    /**
     * @brief Set the ID color for GPU selection rendering
     * @param color RGB color encoding the object ID (values 0-1)
     */
    void SetIdColor(const glm::vec3& color) override { id_color_ = color; }
    
    /**
     * @brief Check if this primitive supports ID rendering
     * @return Always true - all geometric primitives support ID rendering
     */
    bool SupportsIdRendering() const override { return true; }

    // =================================================================
    // Geometry Utility Interface (Pure Virtual)
    // =================================================================

    /**
     * @brief Calculate the volume of the primitive
     * @return Volume in cubic units
     */
    virtual float GetVolume() const = 0;

    /**
     * @brief Calculate the surface area of the primitive
     * @return Surface area in square units
     */
    virtual float GetSurfaceArea() const = 0;

    /**
     * @brief Get the geometric centroid of the primitive
     * @return Centroid position in local space
     */
    virtual glm::vec3 GetCentroid() const = 0;

    // =================================================================
    // OpenGlObject Interface Implementation
    // =================================================================

    /**
     * @brief Main rendering method (Template Method pattern)
     * 
     * This method orchestrates the complete rendering pipeline:
     * 1. Setup shaders and uniforms
     * 2. Handle transparency and blending
     * 3. Call appropriate render method based on mode
     * 4. Handle selection highlighting
     * 
     * Subclasses should NOT override this method. Instead, implement
     * the protected virtual render methods.
     */
    void OnDraw(const glm::mat4& projection, const glm::mat4& view,
                const glm::mat4& coord_transform = glm::mat4(1.0f)) override final;
    
    /**
     * @brief Clean up shared shader programs (public for external cleanup)
     * @note Must be called before OpenGL context is destroyed to prevent segfault
     */
    static void CleanupShaders();

protected:
    /**
     * @brief Protected constructor - only derived classes can instantiate
     */
    GeometricPrimitive();

    // =================================================================
    // Template Method Hooks for Subclasses
    // =================================================================

    /**
     * @brief Prepare shaders and uniforms for rendering
     * @param mvp_matrix Model-View-Projection matrix
     * @param model_matrix Model transformation matrix
     */
    virtual void PrepareShaders(const glm::mat4& mvp_matrix, const glm::mat4& model_matrix) = 0;

    /**
     * @brief Render the primitive in solid mode
     * Called by OnDraw() when render_mode_ == kSolid or kTransparent
     */
    virtual void RenderSolid() = 0;

    /**
     * @brief Render the primitive in wireframe mode
     * Called by OnDraw() when render_mode_ == kWireframe or kOutline
     */
    virtual void RenderWireframe() = 0;

    /**
     * @brief Render the primitive as points
     * Called by OnDraw() when render_mode_ == kPoints
     */
    virtual void RenderPoints() = 0;

    // =================================================================
    // State Management Utilities
    // =================================================================

    /**
     * @brief Mark the primitive as needing GPU buffer updates
     */
    virtual void MarkForUpdate() { needs_update_ = true; }

    /**
     * @brief Check if primitive needs GPU updates
     * @return True if buffers need updating
     */
    virtual bool NeedsUpdate() const { return needs_update_; }

    /**
     * @brief Clear the update flag (called after GPU update)
     */
    virtual void ClearUpdateFlag() { needs_update_ = false; }

    // =================================================================
    // Common Data Members
    // =================================================================

    Material material_;                        ///< Current material properties
    RenderMode render_mode_ = RenderMode::kSolid;  ///< Current rendering mode
    BlendMode blend_mode_ = BlendMode::kOpaque;     ///< Current blending mode
    
    float wireframe_width_ = 1.0f;            ///< Line width for wireframe rendering
    float point_size_ = 1.0f;                 ///< Point size for point rendering
    
    bool needs_update_ = true;                 ///< Dirty flag for GPU updates
    
    // Selection state
    bool is_highlighted_ = false;              ///< Current selection state
    Material original_material_;               ///< Material backup for unhighlighting
    
    // GPU ID-Buffer selection state
    bool id_render_mode_ = false;              ///< True when rendering with ID colors for GPU selection
    glm::vec3 id_color_{0.0f};                 ///< RGB color encoding object ID for GPU selection

    // =================================================================
    // Shared Resources (Static Members)
    // =================================================================

    /**
     * @brief Shared shader programs across all primitive instances
     * 
     * These are initialized once and reused by all primitives for efficiency.
     * The shaders are designed to handle all primitive types through uniforms.
     */
    static std::unique_ptr<ShaderProgram> solid_shader_;      ///< Solid rendering shader
    static std::unique_ptr<ShaderProgram> wireframe_shader_;  ///< Wireframe rendering shader  
    static std::unique_ptr<ShaderProgram> transparent_shader_; ///< Transparent rendering shader
    static std::unique_ptr<ShaderProgram> point_shader_;      ///< Point rendering shader
    
    static bool shaders_initialized_;         ///< Flag to prevent double initialization

    /**
     * @brief Initialize shared shader programs
     * Called automatically on first primitive creation
     */
    static void InitializeShaders();


    // =================================================================
    // OpenGL State Management
    // =================================================================

    /**
     * @brief Setup OpenGL state for the current render mode and blend mode
     */
    virtual void SetupRenderState();

    /**
     * @brief Restore OpenGL state after rendering
     */
    virtual void RestoreRenderState();

private:
    // Prevent copying (geometries should be unique)
    GeometricPrimitive(const GeometricPrimitive&) = delete;
    GeometricPrimitive& operator=(const GeometricPrimitive&) = delete;
};

// =================================================================
// Utility Functions
// =================================================================

/**
 * @brief Convert RenderMode enum to string for debugging
 * @param mode Render mode to convert
 * @return String representation of the mode
 */
const char* RenderModeToString(GeometricPrimitive::RenderMode mode);

/**
 * @brief Convert BlendMode enum to string for debugging
 * @param mode Blend mode to convert  
 * @return String representation of the mode
 */
const char* BlendModeToString(GeometricPrimitive::BlendMode mode);

} // namespace quickviz

#endif // QUICKVIZ_GEOMETRIC_PRIMITIVE_HPP