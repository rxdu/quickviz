/*
 * @file gl_view.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Reusable OpenGL view class for testing renderable objects
 *
 * This class handles the common boilerplate code for setting up OpenGL rendering tests,
 * allowing test cases to focus on creating and configuring renderable objects.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_GLVIEW_HPP
#define QUICKVIZ_GLVIEW_HPP

#include <memory>
#include <string>
#include <vector>
#include <functional>

#include <glm/glm.hpp>

#include "imview/viewer.hpp"
#include "imview/box.hpp"
#include "gldraw/gl_scene_manager.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/coordinate_frame.hpp"

namespace quickviz {

/**
 * @brief Reusable OpenGL view for testing renderable objects
 * 
 * This class encapsulates the common setup and management code needed for
 * OpenGL rendering tests. It provides:
 * - Automatic viewer and scene manager setup
 * - Optional grid and coordinate frame
 * - Scene population callback system
 * - Standard camera controls and help text
 * - Exception handling and error reporting
 */
class GlView {
public:
    /**
     * @brief Configuration structure for GlView
     */
    struct Config {
        std::string window_title;
        bool show_grid;
        bool show_coordinate_frame;
        float grid_size;
        float grid_step;
        glm::vec3 grid_color;
        float coordinate_frame_size;
        GlSceneManager::Mode scene_mode;
        
        Config()
            : window_title("OpenGL Rendering Test")
            , show_grid(true)
            , show_coordinate_frame(true)
            , grid_size(20.0f)
            , grid_step(1.0f)
            , grid_color(0.5f, 0.5f, 0.5f)
            , coordinate_frame_size(1.5f)
            , scene_mode(GlSceneManager::Mode::k3D) {}
    };

    /**
     * @brief Scene setup callback function type
     * 
     * This function is called to populate the scene with renderable objects.
     * It receives a pointer to the scene manager for adding objects.
     */
    using SceneSetupCallback = std::function<void(GlSceneManager*)>;

    /**
     * @brief Constructor
     * 
     * @param config Configuration for the view
     */
    explicit GlView(const Config& config = Config{});

    /**
     * @brief Destructor
     */
    ~GlView() = default;

    // Disable copy construction and assignment
    GlView(const GlView&) = delete;
    GlView& operator=(const GlView&) = delete;

    // Enable move construction and assignment
    GlView(GlView&&) = default;
    GlView& operator=(GlView&&) = default;

    /**
     * @brief Set the scene setup callback
     * 
     * @param callback Function to call for scene setup
     */
    void SetSceneSetup(SceneSetupCallback callback);

    /**
     * @brief Add additional help text to display
     * 
     * @param section_title Title for the help section
     * @param help_lines Vector of help text lines
     */
    void AddHelpSection(const std::string& section_title, 
                       const std::vector<std::string>& help_lines);

    /**
     * @brief Set additional description text
     * 
     * @param description Description to display at startup
     */
    void SetDescription(const std::string& description);

    /**
     * @brief Get access to the scene manager for advanced configuration
     * 
     * @return Pointer to the scene manager
     */
    GlSceneManager* GetSceneManager() const;

    /**
     * @brief Run the view (blocks until window is closed)
     * 
     * This method sets up the scene, displays help information,
     * and runs the main viewer loop.
     */
    void Run();

private:
    /**
     * @brief Set up the viewer and scene manager
     */
    void SetupViewer();

    /**
     * @brief Set up the basic scene elements (grid, coordinate frame)
     */
    void SetupBasicScene();

    /**
     * @brief Display help information
     */
    void DisplayHelp() const;

private:
    Config config_;
    Viewer viewer_;
    std::shared_ptr<GlSceneManager> scene_manager_;
    SceneSetupCallback scene_setup_callback_;
    std::string description_;
    std::vector<std::pair<std::string, std::vector<std::string>>> help_sections_;
};

} // namespace quickviz

#endif // QUICKVIZ_GLVIEW_HPP
