/*
 * @file test_grid.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Test for Grid rendering functionality
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <memory>

#include "gldraw/gl_view.hpp"
#include "gldraw/renderable/grid.hpp"

using namespace quickviz;

void SetupGridScene(GlSceneManager* scene_manager) {
    // Main grid - standard gray
    auto main_grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
    scene_manager->AddOpenGLObject("main_grid", std::move(main_grid));

    // Fine grid - smaller step, lighter color
    auto fine_grid = std::make_unique<Grid>(5.0f, 0.2f, glm::vec3(0.3f, 0.3f, 0.3f));
    scene_manager->AddOpenGLObject("fine_grid", std::move(fine_grid));

    // Large grid - larger step, darker color
    auto large_grid = std::make_unique<Grid>(20.0f, 2.0f, glm::vec3(0.9f, 0.9f, 0.9f));
    scene_manager->AddOpenGLObject("large_grid", std::move(large_grid));

    // Colored grid - red
    auto red_grid = std::make_unique<Grid>(8.0f, 0.5f, glm::vec3(0.8f, 0.2f, 0.2f));
    scene_manager->AddOpenGLObject("red_grid", std::move(red_grid));

    // Colored grid - green
    auto green_grid = std::make_unique<Grid>(6.0f, 0.3f, glm::vec3(0.2f, 0.8f, 0.2f));
    scene_manager->AddOpenGLObject("green_grid", std::move(green_grid));

    // Colored grid - blue
    auto blue_grid = std::make_unique<Grid>(12.0f, 1.5f, glm::vec3(0.2f, 0.2f, 0.8f));
    scene_manager->AddOpenGLObject("blue_grid", std::move(blue_grid));
}

int main(int argc, char* argv[]) {
    try {
        // Configure the view for 3D mode
        GlView::Config config;
        config.window_title = "Grid Rendering Test";
        config.coordinate_frame_size = 2.0f;
        config.show_grid = false;  // We'll add our own grids
        
        // Create the view
        GlView view(config);
        
        // Set up description and help sections
        view.SetDescription("Testing grid rendering with various sizes, steps, and colors");
        
        view.AddHelpSection("Grid Features Demonstrated", {
            "- Multiple overlapping grids",
            "- Different grid sizes (5.0f to 20.0f)",
            "- Various step sizes (0.2f to 2.0f)",
            "- Different colors: gray, red, green, blue",
            "- Fine detail and coarse overview grids",
            "- Alpha blending between overlapping grids"
        });
        
        view.AddHelpSection("Scene Contents", {
            "- Main grid: 10x10 units, 1.0 step, gray",
            "- Fine grid: 5x5 units, 0.2 step, dark gray",
            "- Large grid: 20x20 units, 2.0 step, light gray",
            "- Red grid: 8x8 units, 0.5 step",
            "- Green grid: 6x6 units, 0.3 step", 
            "- Blue grid: 12x12 units, 1.5 step",
            "- Reference coordinate frame"
        });
        
        view.AddHelpSection("Grid Usage", {
            "- Grids provide spatial reference in 3D scenes",
            "- Different step sizes for different levels of detail",
            "- Color coding can indicate different coordinate systems",
            "- Overlapping grids show scale relationships",
            "- Grid lines help with depth perception in 3D",
            "- Useful for robotics path planning visualization"
        });
        
        view.AddHelpSection("Visual Effects", {
            "- Alpha blending allows multiple grid layers",
            "- Depth testing ensures proper 3D rendering",
            "- Fine grids provide detailed reference",
            "- Coarse grids show larger scale structure",
            "- Color coding distinguishes different grid systems"
        });
        
        // Set the scene setup callback
        view.SetSceneSetup(SetupGridScene);
        
        // Run the view
        view.Run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}