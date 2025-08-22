/*
 * @file layer_manager.hpp
 * @date Dec 2024
 * @brief Multi-layer rendering system for point cloud highlighting and visualization
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_LAYER_MANAGER_HPP
#define QUICKVIZ_LAYER_MANAGER_HPP

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <memory>
#include <algorithm>
#include <glm/glm.hpp>

namespace quickviz {

/**
 * @brief Point rendering layer for organizing and highlighting different point groups
 */
class PointLayer {
public:
    enum class BlendMode {
        kReplace,     // Replace base colors completely
        kMultiply,    // Multiply with base colors
        kOverlay,     // Overlay on top of base colors
        kAdditive     // Additive blending
    };

    enum class HighlightMode {
        kColorOnly,        // Only change color
        kSizeIncrease,     // Increase point size
        kOutline,          // Add outline effect
        kGlow,             // Add glow effect
        kColorAndSize,     // Change both color and size
        kSphereSurface     // Color visible sphere surface (for 3D sphere mode)
    };

    PointLayer(const std::string& name, int priority = 0);
    ~PointLayer() = default;

    // Layer properties
    void SetName(const std::string& name) { name_ = name; }
    const std::string& GetName() const { return name_; }
    
    void SetPriority(int priority) { priority_ = priority; }
    int GetPriority() const { return priority_; }
    
    void SetVisible(bool visible) { visible_ = visible; }
    bool IsVisible() const { return visible_; }
    
    void SetOpacity(float opacity) { opacity_ = std::max(0.0f, std::min(opacity, 1.0f)); }
    float GetOpacity() const { return opacity_; }

    // Point management
    void SetPoints(const std::vector<size_t>& point_indices);
    void SetPoints(std::vector<size_t>&& point_indices);
    void AddPoints(const std::vector<size_t>& point_indices);
    void RemovePoints(const std::vector<size_t>& point_indices);
    void ClearPoints();
    
    const std::unordered_set<size_t>& GetPointIndices() const { return point_indices_; }
    size_t GetPointCount() const { return point_indices_.size(); }
    bool ContainsPoint(size_t index) const { return point_indices_.count(index) > 0; }

    // Visual properties
    void SetColor(const glm::vec3& color) { color_ = color; }
    const glm::vec3& GetColor() const { return color_; }
    
    void SetPointSizeMultiplier(float multiplier) { point_size_multiplier_ = multiplier; }
    float GetPointSizeMultiplier() const { return point_size_multiplier_; }
    
    void SetBlendMode(BlendMode mode) { blend_mode_ = mode; }
    BlendMode GetBlendMode() const { return blend_mode_; }
    
    void SetHighlightMode(HighlightMode mode) { highlight_mode_ = mode; }
    HighlightMode GetHighlightMode() const { return highlight_mode_; }

    // Outline/glow effects
    void SetOutlineWidth(float width) { outline_width_ = width; }
    float GetOutlineWidth() const { return outline_width_; }
    
    void SetOutlineColor(const glm::vec3& color) { outline_color_ = color; }
    const glm::vec3& GetOutlineColor() const { return outline_color_; }
    
    void SetGlowIntensity(float intensity) { glow_intensity_ = intensity; }
    float GetGlowIntensity() const { return glow_intensity_; }

private:
    std::string name_;
    int priority_;
    bool visible_;
    float opacity_;
    
    std::unordered_set<size_t> point_indices_;
    
    glm::vec3 color_;
    float point_size_multiplier_;
    BlendMode blend_mode_;
    HighlightMode highlight_mode_;
    
    // Effect properties
    float outline_width_;
    glm::vec3 outline_color_;
    float glow_intensity_;
};

/**
 * @brief Manages multiple rendering layers for point clouds
 */
class LayerManager {
public:
    LayerManager();
    ~LayerManager() = default;

    // Layer management
    std::shared_ptr<PointLayer> CreateLayer(const std::string& name, int priority = 0);
    bool RemoveLayer(const std::string& name);
    bool RemoveLayer(std::shared_ptr<PointLayer> layer);
    void ClearAllLayers();
    
    std::shared_ptr<PointLayer> GetLayer(const std::string& name);
    const std::shared_ptr<PointLayer> GetLayer(const std::string& name) const;
    
    std::vector<std::shared_ptr<PointLayer>> GetAllLayers() const;
    std::vector<std::shared_ptr<PointLayer>> GetVisibleLayers() const;
    std::vector<std::shared_ptr<PointLayer>> GetLayersByPriority() const;
    
    size_t GetLayerCount() const { return layers_.size(); }
    bool HasLayer(const std::string& name) const;

    // Global layer controls
    void SetGlobalOpacity(float opacity) { global_opacity_ = std::max(0.0f, std::min(opacity, 1.0f)); }
    float GetGlobalOpacity() const { return global_opacity_; }
    
    void SetAllLayersVisible(bool visible);
    void SetLayerVisibility(const std::string& name, bool visible);

    // Point queries across layers
    std::vector<std::string> GetLayersContainingPoint(size_t point_index) const;
    std::shared_ptr<PointLayer> GetTopLayerContainingPoint(size_t point_index) const;
    bool IsPointInAnyLayer(size_t point_index) const;

    // Rendering data generation
    struct LayerRenderData {
        std::vector<size_t> point_indices;
        glm::vec3 color;
        float point_size_multiplier;
        float opacity;
        PointLayer::BlendMode blend_mode;
        PointLayer::HighlightMode highlight_mode;
        
        // Effect properties
        float outline_width;
        glm::vec3 outline_color;
        float glow_intensity;
    };

    std::vector<LayerRenderData> GenerateRenderData() const;
    
    // Statistics and debugging
    struct LayerStats {
        size_t total_layers;
        size_t visible_layers;
        size_t total_points_in_layers;
        size_t unique_points_in_layers;
        std::unordered_map<std::string, size_t> layer_point_counts;
    };
    
    LayerStats GetStatistics() const;
    void PrintLayerInfo() const;

private:
    void SortLayersByPriority();
    
    std::vector<std::shared_ptr<PointLayer>> layers_;
    std::unordered_map<std::string, std::shared_ptr<PointLayer>> layer_map_;
    float global_opacity_;
    bool needs_sorting_;
};

} // namespace quickviz

#endif // QUICKVIZ_LAYER_MANAGER_HPP