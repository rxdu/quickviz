/*
 * @file layer_manager.cpp
 * @date Dec 2024
 * @brief Implementation of multi-layer rendering system
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "gldraw/renderable/layer_manager.hpp"
#include <algorithm>
#include <iostream>
#include <limits>

namespace quickviz {

// PointLayer implementation
PointLayer::PointLayer(const std::string& name, int priority)
    : name_(name)
    , priority_(priority)
    , visible_(true)
    , opacity_(1.0f)
    , color_(1.0f, 0.0f, 0.0f)  // Default red
    , point_size_multiplier_(1.0f)
    , blend_mode_(BlendMode::kReplace)
    , highlight_mode_(HighlightMode::kColorOnly)
    , outline_width_(1.0f)
    , outline_color_(1.0f, 1.0f, 1.0f)  // Default white outline
    , glow_intensity_(0.5f) {
}

void PointLayer::SetPoints(const std::vector<size_t>& point_indices) {
    point_indices_.clear();
    point_indices_.insert(point_indices.begin(), point_indices.end());
}

void PointLayer::SetPoints(std::vector<size_t>&& point_indices) {
    point_indices_.clear();
    point_indices_.insert(std::make_move_iterator(point_indices.begin()),
                         std::make_move_iterator(point_indices.end()));
}

void PointLayer::AddPoints(const std::vector<size_t>& point_indices) {
    point_indices_.insert(point_indices.begin(), point_indices.end());
}

void PointLayer::RemovePoints(const std::vector<size_t>& point_indices) {
    for (size_t index : point_indices) {
        point_indices_.erase(index);
    }
}

void PointLayer::ClearPoints() {
    point_indices_.clear();
}

// LayerManager implementation
LayerManager::LayerManager()
    : global_opacity_(1.0f)
    , needs_sorting_(false) {
}

std::shared_ptr<PointLayer> LayerManager::CreateLayer(const std::string& name, int priority) {
    // Check if layer already exists
    if (HasLayer(name)) {
        std::cout << "Warning: Layer '" << name << "' already exists. Returning existing layer." << std::endl;
        return GetLayer(name);
    }
    
    auto layer = std::make_shared<PointLayer>(name, priority);
    layers_.push_back(layer);
    layer_map_[name] = layer;
    needs_sorting_ = true;
    
    return layer;
}

bool LayerManager::RemoveLayer(const std::string& name) {
    auto it = layer_map_.find(name);
    if (it == layer_map_.end()) {
        return false;
    }
    
    auto layer = it->second;
    layer_map_.erase(it);
    
    layers_.erase(std::remove(layers_.begin(), layers_.end(), layer), layers_.end());
    
    return true;
}

bool LayerManager::RemoveLayer(std::shared_ptr<PointLayer> layer) {
    if (!layer) return false;
    return RemoveLayer(layer->GetName());
}

void LayerManager::ClearAllLayers() {
    layers_.clear();
    layer_map_.clear();
    needs_sorting_ = false;
}

std::shared_ptr<PointLayer> LayerManager::GetLayer(const std::string& name) {
    auto it = layer_map_.find(name);
    return (it != layer_map_.end()) ? it->second : nullptr;
}

const std::shared_ptr<PointLayer> LayerManager::GetLayer(const std::string& name) const {
    auto it = layer_map_.find(name);
    return (it != layer_map_.end()) ? it->second : nullptr;
}

std::vector<std::shared_ptr<PointLayer>> LayerManager::GetAllLayers() const {
    return layers_;
}

std::vector<std::shared_ptr<PointLayer>> LayerManager::GetVisibleLayers() const {
    std::vector<std::shared_ptr<PointLayer>> visible_layers;
    for (const auto& layer : layers_) {
        if (layer && layer->IsVisible()) {
            visible_layers.push_back(layer);
        }
    }
    return visible_layers;
}

std::vector<std::shared_ptr<PointLayer>> LayerManager::GetLayersByPriority() const {
    auto sorted_layers = layers_;
    std::sort(sorted_layers.begin(), sorted_layers.end(),
              [](const std::shared_ptr<PointLayer>& a, const std::shared_ptr<PointLayer>& b) {
                  return a->GetPriority() < b->GetPriority();
              });
    return sorted_layers;
}

bool LayerManager::HasLayer(const std::string& name) const {
    return layer_map_.find(name) != layer_map_.end();
}

void LayerManager::SetAllLayersVisible(bool visible) {
    for (auto& layer : layers_) {
        if (layer) {
            layer->SetVisible(visible);
        }
    }
}

void LayerManager::SetLayerVisibility(const std::string& name, bool visible) {
    auto layer = GetLayer(name);
    if (layer) {
        layer->SetVisible(visible);
    }
}

std::vector<std::string> LayerManager::GetLayersContainingPoint(size_t point_index) const {
    std::vector<std::string> containing_layers;
    for (const auto& layer : layers_) {
        if (layer && layer->ContainsPoint(point_index)) {
            containing_layers.push_back(layer->GetName());
        }
    }
    return containing_layers;
}

std::shared_ptr<PointLayer> LayerManager::GetTopLayerContainingPoint(size_t point_index) const {
    std::shared_ptr<PointLayer> top_layer = nullptr;
    int highest_priority = std::numeric_limits<int>::min();
    
    for (const auto& layer : layers_) {
        if (layer && layer->IsVisible() && layer->ContainsPoint(point_index)) {
            if (layer->GetPriority() > highest_priority) {
                highest_priority = layer->GetPriority();
                top_layer = layer;
            }
        }
    }
    
    return top_layer;
}

bool LayerManager::IsPointInAnyLayer(size_t point_index) const {
    for (const auto& layer : layers_) {
        if (layer && layer->ContainsPoint(point_index)) {
            return true;
        }
    }
    return false;
}

std::vector<LayerManager::LayerRenderData> LayerManager::GenerateRenderData() const {
    std::vector<LayerRenderData> render_data;
    
    auto sorted_layers = GetLayersByPriority();
    for (const auto& layer : sorted_layers) {
        if (!layer || !layer->IsVisible() || layer->GetPointCount() == 0) {
            continue;
        }
        
        LayerRenderData data;
        data.point_indices.assign(layer->GetPointIndices().begin(), 
                                 layer->GetPointIndices().end());
        data.color = layer->GetColor();
        data.point_size_multiplier = layer->GetPointSizeMultiplier();
        data.opacity = layer->GetOpacity() * global_opacity_;
        data.blend_mode = layer->GetBlendMode();
        data.highlight_mode = layer->GetHighlightMode();
        data.outline_width = layer->GetOutlineWidth();
        data.outline_color = layer->GetOutlineColor();
        data.glow_intensity = layer->GetGlowIntensity();
        
        render_data.push_back(data);
    }
    
    return render_data;
}

LayerManager::LayerStats LayerManager::GetStatistics() const {
    LayerStats stats;
    stats.total_layers = layers_.size();
    stats.visible_layers = 0;
    stats.total_points_in_layers = 0;
    
    std::unordered_set<size_t> unique_points;
    
    for (const auto& layer : layers_) {
        if (!layer) continue;
        
        if (layer->IsVisible()) {
            stats.visible_layers++;
        }
        
        size_t layer_point_count = layer->GetPointCount();
        stats.total_points_in_layers += layer_point_count;
        stats.layer_point_counts[layer->GetName()] = layer_point_count;
        
        const auto& indices = layer->GetPointIndices();
        unique_points.insert(indices.begin(), indices.end());
    }
    
    stats.unique_points_in_layers = unique_points.size();
    
    return stats;
}

void LayerManager::PrintLayerInfo() const {
    std::cout << "\n=== Layer Manager Status ===" << std::endl;
    std::cout << "Total layers: " << layers_.size() << std::endl;
    std::cout << "Global opacity: " << global_opacity_ << std::endl;
    
    auto stats = GetStatistics();
    std::cout << "Visible layers: " << stats.visible_layers << std::endl;
    std::cout << "Total points in layers: " << stats.total_points_in_layers << std::endl;
    std::cout << "Unique points in layers: " << stats.unique_points_in_layers << std::endl;
    
    std::cout << "\nLayer details:" << std::endl;
    auto sorted_layers = GetLayersByPriority();
    for (const auto& layer : sorted_layers) {
        if (!layer) continue;
        
        std::cout << "  - " << layer->GetName() 
                  << " (Priority: " << layer->GetPriority()
                  << ", Points: " << layer->GetPointCount()
                  << ", Visible: " << (layer->IsVisible() ? "Yes" : "No")
                  << ", Opacity: " << layer->GetOpacity() << ")" << std::endl;
    }
    std::cout << "========================\n" << std::endl;
}

void LayerManager::SortLayersByPriority() {
    if (!needs_sorting_) return;
    
    std::sort(layers_.begin(), layers_.end(),
              [](const std::shared_ptr<PointLayer>& a, const std::shared_ptr<PointLayer>& b) {
                  return a->GetPriority() < b->GetPriority();
              });
    
    needs_sorting_ = false;
}

} // namespace quickviz