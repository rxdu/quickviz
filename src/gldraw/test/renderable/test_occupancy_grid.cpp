/*
 * @file test_occupancy_grid.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Test for OccupancyGrid rendering functionality
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include <random>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "gldraw/gl_view.hpp"
#include "gldraw/renderable/occupancy_grid.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/coordinate_frame.hpp"

using namespace quickviz;

void SetupOccupancyGridScene(GlSceneManager* scene_manager) {
    // Add base grid for reference
    auto grid = std::make_unique<Grid>(20.0f, 1.0f, glm::vec3(0.2f, 0.2f, 0.2f));
    scene_manager->AddOpenGLObject("reference_grid", std::move(grid));
    
    // Add coordinate frame for reference
    auto frame = std::make_unique<CoordinateFrame>(2.0f);
    scene_manager->AddOpenGLObject("frame", std::move(frame));
    
    // 1. Basic occupancy map - binary black/white
    auto grid1 = std::make_unique<OccupancyGrid>(20, 20, 0.2f);
    grid1->SetOrigin(glm::vec3(-6.0f, -6.0f, 0.01f));
    std::vector<float> binary_data(400);
    for (size_t i = 0; i < 400; ++i) {
        size_t x = i % 20;
        size_t y = i / 20;
        // Create simple rooms and corridors pattern
        if ((x < 3 || x > 16) || (y < 3 || y > 16)) {
            binary_data[i] = 1.0f;  // Walls
        } else if ((x >= 8 && x <= 11) && (y >= 8 && y <= 11)) {
            binary_data[i] = 1.0f;  // Central obstacle
        } else if ((x == 9 || x == 10) && (y >= 3 && y <= 7)) {
            binary_data[i] = 1.0f;  // Vertical wall
        } else {
            binary_data[i] = 0.0f;  // Free space
        }
    }
    grid1->SetData(binary_data);
    grid1->SetRenderMode(OccupancyGrid::RenderMode::kFlat2D);
    grid1->SetColorMode(OccupancyGrid::ColorMode::kOccupancy);
    grid1->SetCellShape(OccupancyGrid::CellShape::kSquare);
    grid1->SetOccupiedColor(glm::vec3(0.1f, 0.1f, 0.1f));  // Dark gray
    grid1->SetFreeColor(glm::vec3(0.9f, 0.9f, 0.9f));      // Light gray
    scene_manager->AddOpenGLObject("grid_binary", std::move(grid1));
    
    // 2. Probabilistic occupancy map - grayscale
    auto grid2 = std::make_unique<OccupancyGrid>(15, 15, 0.25f);
    grid2->SetOrigin(glm::vec3(2.0f, -4.0f, 0.01f));
    std::vector<float> prob_data(225);
    std::random_device rd;
    std::mt19937 gen(42); // Fixed seed for reproducibility
    std::uniform_real_distribution<> dis(0.0, 1.0);
    
    // Generate probabilistic data with smooth regions
    for (size_t y = 0; y < 15; ++y) {
        for (size_t x = 0; x < 15; ++x) {
            float center_x = 7.5f, center_y = 7.5f;
            float dist = std::sqrt((x - center_x) * (x - center_x) + (y - center_y) * (y - center_y));
            
            if (dist < 3.0f) {
                prob_data[y * 15 + x] = 0.1f + dis(gen) * 0.2f; // Mostly free
            } else if (dist < 6.0f) {
                prob_data[y * 15 + x] = 0.3f + dis(gen) * 0.4f; // Uncertain
            } else {
                prob_data[y * 15 + x] = 0.7f + dis(gen) * 0.3f; // Mostly occupied
            }
        }
    }
    grid2->SetData(prob_data);
    grid2->SetRenderMode(OccupancyGrid::RenderMode::kFlat2D);
    grid2->SetColorMode(OccupancyGrid::ColorMode::kProbability);
    grid2->SetCellShape(OccupancyGrid::CellShape::kCircle);
    scene_manager->AddOpenGLObject("grid_probabilistic", std::move(grid2));
    
    // 3. Cost map - blue to red gradient
    auto grid3 = std::make_unique<OccupancyGrid>(18, 18, 0.2f);
    grid3->SetOrigin(glm::vec3(-8.0f, 2.0f, 0.01f));
    std::vector<float> cost_data(324);
    
    // Create cost map with obstacles having high cost
    for (size_t y = 0; y < 18; ++y) {
        for (size_t x = 0; x < 18; ++x) {
            float cost = 0.0f;
            
            // High cost near borders
            float border_dist = std::min({x, y, 17-x, 17-y});
            if (border_dist < 2) {
                cost = std::max(cost, 0.8f - border_dist * 0.2f);
            }
            
            // Circular obstacles with gradient cost
            std::vector<glm::vec2> obstacles = {
                glm::vec2(5, 5), glm::vec2(12, 12), glm::vec2(5, 12), glm::vec2(12, 5)
            };
            
            for (const auto& obs : obstacles) {
                float obs_dist = glm::length(glm::vec2(x, y) - obs);
                if (obs_dist < 4.0f) {
                    float obs_cost = 1.0f - (obs_dist / 4.0f);
                    cost = std::max(cost, obs_cost);
                }
            }
            
            cost_data[y * 18 + x] = glm::clamp(cost, 0.0f, 1.0f);
        }
    }
    grid3->SetData(cost_data);
    grid3->SetRenderMode(OccupancyGrid::RenderMode::kFlat2D);
    grid3->SetColorMode(OccupancyGrid::ColorMode::kCostmap);
    grid3->SetCellShape(OccupancyGrid::CellShape::kSquare);
    grid3->SetShowGrid(true);
    grid3->SetGridColor(glm::vec3(0.3f, 0.3f, 0.3f));
    grid3->SetGridLineWidth(1.0f);
    scene_manager->AddOpenGLObject("grid_costmap", std::move(grid3));
    
    // 4. Height map - 3D elevation data
    auto grid4 = std::make_unique<OccupancyGrid>(16, 16, 0.3f);
    grid4->SetOrigin(glm::vec3(2.0f, 2.0f, 0.0f));
    std::vector<float> height_data(256);
    
    // Generate terrain-like height data
    for (size_t y = 0; y < 16; ++y) {
        for (size_t x = 0; x < 16; ++x) {
            float fx = static_cast<float>(x) / 16.0f;
            float fy = static_cast<float>(y) / 16.0f;
            
            // Multi-octave noise simulation
            float height = 0.3f * std::sin(fx * M_PI * 4) * std::cos(fy * M_PI * 3);
            height += 0.2f * std::sin(fx * M_PI * 8) * std::sin(fy * M_PI * 6);
            height += 0.1f * std::sin(fx * M_PI * 16) * std::cos(fy * M_PI * 12);
            
            // Normalize to 0-1 range
            height = (height + 0.6f) / 1.2f;
            height_data[y * 16 + x] = glm::clamp(height, 0.0f, 1.0f);
        }
    }
    grid4->SetData(height_data);
    grid4->SetRenderMode(OccupancyGrid::RenderMode::kHeightmap);
    grid4->SetColorMode(OccupancyGrid::ColorMode::kHeight);
    grid4->SetHeightScale(2.0f);
    grid4->SetCellShape(OccupancyGrid::CellShape::kSquare);
    scene_manager->AddOpenGLObject("grid_heightmap", std::move(grid4));
    
    // 5. Hexagonal grid with semantic colors
    auto grid5 = std::make_unique<OccupancyGrid>(12, 10, 0.35f);
    grid5->SetOrigin(glm::vec3(-7.0f, -2.0f, 1.5f));
    std::vector<float> semantic_data(120);
    std::vector<glm::vec3> semantic_colors = {
        glm::vec3(0.2f, 0.8f, 0.2f),  // Green - vegetation
        glm::vec3(0.8f, 0.6f, 0.4f),  // Brown - dirt/ground
        glm::vec3(0.4f, 0.4f, 0.8f),  // Blue - water
        glm::vec3(0.7f, 0.7f, 0.7f),  // Gray - concrete
        glm::vec3(0.8f, 0.2f, 0.2f)   // Red - danger zone
    };
    grid5->SetColorMap(semantic_colors);
    
    // Create semantic pattern
    for (size_t y = 0; y < 10; ++y) {
        for (size_t x = 0; x < 12; ++x) {
            float value;
            if (y < 2) value = 0.8f;        // Concrete (top)
            else if (y < 4) value = 0.2f;   // Vegetation
            else if (y < 6) value = 0.4f;   // Ground
            else if (y < 8) value = 0.6f;   // Water
            else value = 1.0f;              // Danger zone (bottom)
            
            // Add some variation
            if ((x + y) % 3 == 0) value = glm::clamp(value + 0.1f, 0.0f, 1.0f);
            
            semantic_data[y * 12 + x] = value;
        }
    }
    grid5->SetData(semantic_data);
    grid5->SetRenderMode(OccupancyGrid::RenderMode::kFlat2D);
    grid5->SetColorMode(OccupancyGrid::ColorMode::kCustom);
    grid5->SetCellShape(OccupancyGrid::CellShape::kHexagon);
    grid5->SetBorderWidth(2.0f);
    grid5->SetBorderColor(glm::vec3(0.0f, 0.0f, 0.0f));
    scene_manager->AddOpenGLObject("grid_semantic", std::move(grid5));
    
    // 6. Large sparse grid with subsampling
    auto grid6 = std::make_unique<OccupancyGrid>(20, 15, 0.2f);  // Smaller grid for debugging
    grid6->SetOrigin(glm::vec3(8.0f, -6.0f, 0.01f));
    std::vector<float> sparse_data(300);  // 20x15 = 300
    
    // Create clear checkerboard pattern for debugging
    for (size_t y = 0; y < 15; ++y) {
        for (size_t x = 0; x < 20; ++x) {
            if ((x + y) % 3 == 0) {
                sparse_data[y * 20 + x] = 1.0f;  // Occupied (black)
            } else if ((x + y) % 3 == 1) {
                sparse_data[y * 20 + x] = 0.0f;  // Free (white)
            } else {
                sparse_data[y * 20 + x] = 0.5f;  // Uncertain (gray)
            }
        }
    }
    
    grid6->SetData(sparse_data);
    grid6->SetRenderMode(OccupancyGrid::RenderMode::kFlat2D);
    grid6->SetColorMode(OccupancyGrid::ColorMode::kOccupancy);
    grid6->SetSubsampling(1);  // Show all cells
    grid6->SetValueThreshold(-1.0f);  // Show all cells (disable threshold)
    grid6->SetOccupiedColor(glm::vec3(0.0f, 0.0f, 0.0f));  // Black
    grid6->SetFreeColor(glm::vec3(1.0f, 1.0f, 1.0f));      // White
    scene_manager->AddOpenGLObject("grid_sparse", std::move(grid6));
    
    // 7. Multi-layer voxel grid - representing a 3-floor building
    auto grid7 = std::make_unique<OccupancyGrid>(10, 10, 0.4f);
    grid7->SetOrigin(glm::vec3(-4.0f, 6.0f, 0.0f));
    grid7->SetLayerCount(3);
    
    // Layer 0: Ground floor - walls and lobby area
    std::vector<float> layer0(100, 0.0f);
    for (size_t i = 0; i < 100; ++i) {
        size_t x = i % 10;
        size_t y = i / 10;
        // Outer walls
        if (x == 0 || x == 9 || y == 0 || y == 9) {
            layer0[i] = 1.0f;
        }
        // Internal walls creating rooms
        else if ((x == 3 || x == 6) && y > 0 && y < 9) {
            layer0[i] = 0.9f;
        }
        // Reception desk area
        else if (x >= 4 && x <= 5 && y == 2) {
            layer0[i] = 0.8f;
        }
    }
    grid7->SetLayerData(0, layer0);
    grid7->SetLayerHeight(0, 0.0f);
    
    // Layer 1: Second floor - office layout
    std::vector<float> layer1(100, 0.0f);
    for (size_t i = 0; i < 100; ++i) {
        size_t x = i % 10;
        size_t y = i / 10;
        // Outer walls
        if (x == 0 || x == 9 || y == 0 || y == 9) {
            layer1[i] = 1.0f;
        }
        // Corridor in the middle
        else if (y == 5 && x > 0 && x < 9) {
            layer1[i] = 0.0f;  // Keep corridor free
        }
        // Office cubicles
        else if ((x == 2 || x == 4 || x == 6) && (y == 2 || y == 7)) {
            layer1[i] = 0.7f;
        }
        // Meeting room walls
        else if (x == 7 && y >= 2 && y <= 4) {
            layer1[i] = 0.85f;
        }
    }
    grid7->SetLayerData(1, layer1);
    grid7->SetLayerHeight(1, 1.0f);
    
    // Layer 2: Top floor - open plan with pillars
    std::vector<float> layer2(100, 0.0f);
    for (size_t i = 0; i < 100; ++i) {
        size_t x = i % 10;
        size_t y = i / 10;
        // Outer walls only
        if (x == 0 || x == 9 || y == 0 || y == 9) {
            layer2[i] = 1.0f;
        }
        // Support pillars
        else if ((x == 3 || x == 6) && (y == 3 || y == 6)) {
            layer2[i] = 0.95f;
        }
        // Central elevator/stairs
        else if (x >= 4 && x <= 5 && y >= 4 && y <= 5) {
            layer2[i] = 0.9f;
        }
    }
    grid7->SetLayerData(2, layer2);
    grid7->SetLayerHeight(2, 2.0f);
    grid7->SetLayerOpacity(2, 0.8f);
    
    grid7->SetRenderMode(OccupancyGrid::RenderMode::kVoxels);
    grid7->SetColorMode(OccupancyGrid::ColorMode::kSemantic);
    grid7->SetColorMap({glm::vec3(0.3f, 0.3f, 0.3f),   // Gray for layer 0
                        glm::vec3(0.3f, 0.8f, 0.3f),   // Green for layer 1
                        glm::vec3(0.3f, 0.3f, 0.8f)});  // Blue for layer 2
    scene_manager->AddOpenGLObject("grid_voxel", std::move(grid7));
    
    // 8. Transparent grid with animation effect
    auto grid8 = std::make_unique<OccupancyGrid>(10, 10, 0.3f);  // Smaller for debugging
    grid8->SetOrigin(glm::vec3(6.0f, 4.0f, 2.0f));
    std::vector<float> animated_data(100);
    
    // Create clear radial pattern for debugging
    for (size_t y = 0; y < 10; ++y) {
        for (size_t x = 0; x < 10; ++x) {
            float cx = static_cast<float>(x) - 4.5f;
            float cy = static_cast<float>(y) - 4.5f; 
            float dist = std::sqrt(cx*cx + cy*cy);
            
            if (dist < 2.0f) {
                animated_data[y * 10 + x] = 0.0f;  // Blue center
            } else if (dist < 3.5f) {
                animated_data[y * 10 + x] = 0.5f;  // Green middle
            } else {
                animated_data[y * 10 + x] = 1.0f;  // Red outer
            }
        }
    }
    grid8->SetData(animated_data);
    grid8->SetRenderMode(OccupancyGrid::RenderMode::kFlat2D);
    grid8->SetColorMode(OccupancyGrid::ColorMode::kCostmap);
    grid8->SetCellShape(OccupancyGrid::CellShape::kSquare);  // Use squares for clarity
    grid8->SetTransparency(1.0f);  // Fully opaque
    grid8->SetShowGrid(true);
    grid8->SetGridColor(glm::vec3(0.0f, 0.0f, 0.0f));  // Black grid lines
    grid8->SetGridLineWidth(2.0f);
    scene_manager->AddOpenGLObject("grid_animated", std::move(grid8));
}

int main(int argc, char* argv[]) {
    try {
        // Configure the view for 3D mode
        GlView::Config config;
        config.window_title = "Occupancy Grid Rendering Test";
        config.coordinate_frame_size = 2.0f;
        
        // Create the view
        GlView view(config);
        
        // Set up description and help sections
        view.SetDescription("Testing occupancy grid rendering for SLAM maps and spatial data visualization");
        
        view.AddHelpSection("Occupancy Grid Features Demonstrated", {
            "- Multiple render modes: flat 2D, 3D heightmap, voxels, contours",
            "- Color encoding: occupancy, probability, cost, height, semantic",
            "- Cell shapes: square, circle, hexagon for different visual styles",
            "- Multi-layer support for 3D volumetric data",
            "- Performance optimizations: subsampling, LOD, thresholding",
            "- Visual enhancements: grid lines, borders, transparency",
            "- Large grid handling with efficient GPU rendering"
        });
        
        view.AddHelpSection("Scene Contents", {
            "- Binary occupancy map: Black/white room layout with walls and obstacles",
            "- Probabilistic map: Grayscale circular cells showing uncertainty",
            "- Cost map: Blue-to-red gradient showing navigation costs with grid lines",
            "- Height map: 3D terrain visualization with rainbow height coloring",
            "- Semantic hexagon grid: Color-coded terrain types (vegetation, water, etc.)",
            "- Sparse large grid: Efficient rendering with subsampling for big datasets",
            "- Multi-layer voxels: 3D volumetric data with different floor levels",
            "- Animated transparent grid: Wave pattern with transparency and grid overlay"
        });
        
        view.AddHelpSection("Robotics Applications", {
            "- SLAM mapping and occupancy grid visualization",
            "- Navigation cost map display for path planning",
            "- Multi-floor building maps and 3D environments",
            "- Terrain analysis and elevation mapping",
            "- Sensor coverage and detection probability maps",
            "- Semantic mapping for object recognition",
            "- Dynamic map updates and real-time visualization",
            "- Large-scale outdoor mapping with LOD optimization"
        });
        
        view.AddHelpSection("Render Mode Details", {
            "- kFlat2D: Standard 2D occupancy grid at fixed height",
            "- kHeightmap: 3D boxes with height representing values",
            "- kVoxels: Multi-layer 3D volumetric representation",
            "- kContour: Contour lines for elevation and gradient display",
            "- Configurable cell resolution and world coordinates",
            "- Support for both metric and pixel-based grids"
        });
        
        view.AddHelpSection("Color Mode Options", {
            "- kOccupancy: Binary black/white for occupied/free space",
            "- kProbability: Grayscale gradient for uncertainty values",
            "- kCostmap: Blue-to-red for navigation cost visualization",
            "- kHeight: Rainbow gradient for elevation data",
            "- kSemantic: Custom colors for semantic class mapping",
            "- kCustom: User-defined color palettes and mappings"
        });
        
        view.AddHelpSection("Performance Features", {
            "- Subsampling: Render every Nth cell for large grids",
            "- Value thresholding: Hide cells below minimum threshold",
            "- Level of detail: Automatic quality reduction at distance",
            "- GPU-optimized rendering with batched draw calls",
            "- Efficient memory management for dynamic updates",
            "- Support for sparse grids with many empty cells"
        });
        
        view.AddHelpSection("Data Formats Supported", {
            "- Standard float arrays (0.0-1.0 probability values)",
            "- ROS occupancy_msgs format (-1=unknown, 0-100=probability)",
            "- Multi-layer data for 3D volumetric grids",
            "- Real-time updates for dynamic mapping",
            "- Coordinate system integration with world transforms"
        });
        
        view.AddHelpSection("API Usage Examples", {
            "grid->SetGridSize(width, height)  // Set grid dimensions",
            "grid->SetResolution(0.1f)  // 10cm per cell",
            "grid->SetData(occupancy_data)  // Load probability values",
            "grid->SetRenderMode(OccupancyGrid::RenderMode::kHeightmap)",
            "grid->SetColorMode(OccupancyGrid::ColorMode::kCostmap)",
            "grid->SetShowGrid(true)  // Enable grid line overlay",
            "grid->SetSubsampling(2)  // Show every 2nd cell for performance",
            "grid->SetValueThreshold(0.1f)  // Hide low-probability cells"
        });
        
        // Set the scene setup callback
        view.SetSceneSetup(SetupOccupancyGridScene);
        
        // Run the view
        view.Run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}