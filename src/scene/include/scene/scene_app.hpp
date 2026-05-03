/*
 * @file scene_app.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Five-line quickstart for a 3D viewer
 *
 * `SceneApp` is the smallest entry point QuickViz offers for building a
 * 3D viewer. It wraps a `Viewer` and a `GlScenePanel`, sets up sensible
 * defaults (grid + coordinate frame), and runs the event loop. The
 * scene is populated through a callback the caller provides.
 *
 * Typical use:
 *
 *     quickviz::SceneApp app({.window_title = "My Tool"});
 *     app.SetSceneSetup([](SceneManager* scene) {
 *       scene->AddOpenGLObject("cloud", std::make_unique<PointCloud>(...));
 *     });
 *     app.Run();
 *
 * For more control (multiple panels, side widgets, plot/canvas/image
 * panels alongside the 3D scene) drop down to `Viewer` directly. SceneApp
 * is the path of least resistance, not the only door.
 *
 * Internally also used by the renderable unit tests in scene/test/.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_SCENE_APP_HPP
#define QUICKVIZ_SCENE_APP_HPP

#include <memory>
#include <string>
#include <vector>
#include <functional>

#include <glm/glm.hpp>

#include "viewer/viewer.hpp"
#include "viewer/box.hpp"
#include "scene/gl_scene_panel.hpp"
#include "scene/renderable/grid.hpp"
#include "scene/renderable/coordinate_frame.hpp"

namespace quickviz {

/**
 * @brief Quickstart facade for building a 3D viewer in a few lines
 *
 * Provides:
 * - `Viewer` + `GlScenePanel` set up with sane defaults
 * - Optional reference grid and coordinate frame
 * - A `SceneSetupCallback` for populating the scene
 * - Standard camera controls (inherited from `GlScenePanel`)
 * - Optional help-text overlay
 *
 * For applications that need multiple panels (plots, image views, custom
 * UI) compose `Viewer` and the panel classes directly — `SceneApp` is a
 * convenience layer, not a base class to extend.
 */
class SceneApp {
public:
    /**
     * @brief Configuration for the SceneApp
     */
    struct Config {
        std::string window_title;
        bool show_grid;
        bool show_coordinate_frame;
        float grid_size;
        float grid_step;
        glm::vec3 grid_color;
        float coordinate_frame_size;
        SceneManager::Mode scene_mode;
        
        Config()
            : window_title("OpenGL Rendering Test")
            , show_grid(true)
            , show_coordinate_frame(true)
            , grid_size(20.0f)
            , grid_step(1.0f)
            , grid_color(0.5f, 0.5f, 0.5f)
            , coordinate_frame_size(1.5f)
            , scene_mode(SceneManager::Mode::k3D) {}
    };

    /**
     * @brief Scene setup callback function type
     * 
     * This function is called to populate the scene with renderable objects.
     * It receives a pointer to the scene manager for adding objects.
     */
    using SceneSetupCallback = std::function<void(SceneManager*)>;

    /**
     * @brief Constructor
     * 
     * @param config Configuration for the view
     */
    explicit SceneApp(const Config& config = Config{});

    /**
     * @brief Destructor
     */
    ~SceneApp() = default;

    // Disable copy construction and assignment
    SceneApp(const SceneApp&) = delete;
    SceneApp& operator=(const SceneApp&) = delete;

    // Enable move construction and assignment
    SceneApp(SceneApp&&) = default;
    SceneApp& operator=(SceneApp&&) = default;

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
    SceneManager* GetSceneManager() const;

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
    std::shared_ptr<GlScenePanel> scene_panel_;
    SceneSetupCallback scene_setup_callback_;
    std::string description_;
    std::vector<std::pair<std::string, std::vector<std::string>>> help_sections_;
};

} // namespace quickviz

#endif // QUICKVIZ_SCENE_APP_HPP
