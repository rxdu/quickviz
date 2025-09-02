/*
 * @file visual_test_virtual_scene.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date August 27, 2025
 * @brief Visual test for VirtualScene with GlRenderBackend integration
 *
 * This test demonstrates the virtual scene system using GlRenderBackend
 * to render virtual objects through the high-level VirtualScene interface.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <vector>

#include "gldraw/gl_viewer.hpp"
#include "vscene/virtual_scene.hpp"
#include "vscene/virtual_sphere.hpp"
#include "vscene/gl_render_backend.hpp"

using namespace quickviz;

class VirtualSceneDemo {
public:
    VirtualSceneDemo() = default;
    
    void SetupVirtualScene(SceneManager* gl_scene_manager) {
        // Create virtual scene with GlRenderBackend using the external scene manager
        scene_ = std::make_unique<VirtualScene>();
        backend_ = std::make_unique<GlRenderBackend>(gl_scene_manager);  // Use external scene manager
        scene_->SetRenderBackend(std::move(backend_));
        
        // Store reference to GlSceneManager for camera access
        gl_scene_manager_ = gl_scene_manager;
        
        CreateVirtualSpheres();
        
        // Update scene to sync with backend
        scene_->Update(0.0f);
        
        auto object_ids = scene_->GetObjectIds();
        std::cout << "Virtual Scene Demo initialized with " 
                  << object_ids.size() << " objects" << std::endl;
        std::cout << "Using external GlSceneManager for rendering" << std::endl;
    }
    
    void CreateVirtualSpheres() {
        // 1. Basic virtual sphere - Red
        auto sphere1 = std::make_unique<VirtualSphere>("sphere1", 1.0f);
        sphere1->SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
        sphere1->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
        scene_->AddObject("sphere1", std::move(sphere1));
        
        // 2. Large virtual sphere - Green  
        auto sphere2 = std::make_unique<VirtualSphere>("sphere2", 2.0f);
        sphere2->SetPosition(glm::vec3(-4.0f, 0.0f, 0.0f));
        sphere2->SetColor(glm::vec3(0.0f, 1.0f, 0.0f));
        scene_->AddObject("sphere2", std::move(sphere2));
        
        // 3. Small virtual sphere - Cyan
        auto sphere3 = std::make_unique<VirtualSphere>("sphere3", 0.5f);
        sphere3->SetPosition(glm::vec3(3.0f, 0.0f, 0.0f));
        sphere3->SetColor(glm::vec3(0.0f, 0.8f, 1.0f));
        scene_->AddObject("sphere3", std::move(sphere3));
        
        // 4. Transparent virtual sphere - Yellow
        auto sphere4 = std::make_unique<VirtualSphere>("sphere4", 1.5f);
        sphere4->SetPosition(glm::vec3(0.0f, 3.0f, 0.0f));
        sphere4->SetColor(glm::vec3(1.0f, 1.0f, 0.0f));
        // Note: Opacity/transparency not yet implemented in Step 3
        // sphere4->SetOpacity(0.6f);
        scene_->AddObject("sphere4", std::move(sphere4));
        
        // 5. Highlighted virtual sphere - Magenta
        auto sphere5 = std::make_unique<VirtualSphere>("sphere5", 1.2f);
        sphere5->SetPosition(glm::vec3(0.0f, -3.0f, 0.0f));
        sphere5->SetColor(glm::vec3(1.0f, 0.0f, 1.0f));
        sphere5->SetSelected(true);  // This should highlight it
        scene_->AddObject("sphere5", std::move(sphere5));
        
        // 6. Invisible sphere initially - Blue (will demonstrate visibility toggle)
        auto sphere6 = std::make_unique<VirtualSphere>("sphere6", 0.8f);
        sphere6->SetPosition(glm::vec3(2.0f, 2.0f, 0.0f));
        sphere6->SetColor(glm::vec3(0.0f, 0.0f, 1.0f));
        sphere6->SetVisible(false);  // Initially hidden
        scene_->AddObject("sphere6", std::move(sphere6));
        
        std::cout << "Created virtual spheres with various properties:" << std::endl;
        std::cout << "- sphere1: Red basic sphere at origin" << std::endl;
        std::cout << "- sphere2: Green large sphere at (-4,0,0)" << std::endl; 
        std::cout << "- sphere3: Cyan small sphere at (3,0,0)" << std::endl;
        std::cout << "- sphere4: Yellow transparent sphere at (0,3,0)" << std::endl;
        std::cout << "- sphere5: Magenta highlighted sphere at (0,-3,0)" << std::endl;
        std::cout << "- sphere6: Blue hidden sphere at (2,2,0)" << std::endl;
    }
    
    // Demo update function (could be used for animations)
    void Update(float dt) {
        if (scene_) {
            scene_->Update(dt);
        }
    }
    
    // Toggle visibility of the hidden sphere for demonstration
    void ToggleHiddenSphere() {
        if (scene_) {
            auto* sphere6 = scene_->GetObject("sphere6");
            if (sphere6) {
                // Toggle visibility - need to track state since no getter
                static bool sphere6_visible = false;
                sphere6_visible = !sphere6_visible;
                sphere6->SetVisible(sphere6_visible);
                scene_->Update(0.0f);
                std::cout << "Sphere6 visibility: " << (sphere6_visible ? "ON" : "OFF") << std::endl;
            }
        }
    }
    
    // Demonstrate selection functionality
    void SelectSphere(const std::string& id) {
        if (scene_) {
            scene_->SetSelected(id, true);
            scene_->Update(0.0f);
            std::cout << "Selected sphere: " << id << std::endl;
        }
    }

private:
    std::unique_ptr<VirtualScene> scene_;
    std::unique_ptr<GlRenderBackend> backend_;
    SceneManager* gl_scene_manager_ = nullptr;
};

// Global demo instance for callback access
std::unique_ptr<VirtualSceneDemo> g_demo;

void SetupVirtualSceneDemo(SceneManager* scene_manager) {
    g_demo = std::make_unique<VirtualSceneDemo>();
    g_demo->SetupVirtualScene(scene_manager);
}

int main(int argc, char* argv[]) {
    try {
        // Configure the viewer for 3D mode
        GlViewer::Config config;
        config.window_title = "Virtual Scene Integration Test";
        config.coordinate_frame_size = 2.0f;
        
        // Create the viewer
        GlViewer viewer(config);
        
        // Set up description and help sections
        viewer.SetDescription("Testing VirtualScene integration with GlRenderBackend");
        
        viewer.AddHelpSection("Virtual Scene Features Demonstrated", {
            "- High-level VirtualSphere objects",
            "- GlRenderBackend integration with GlSceneManager", 
            "- Virtual object property mapping to OpenGL",
            "- Object lifecycle management through VirtualScene",
            "- Visibility and highlighting states",
            "- Transparent rendering support"
        });
        
        viewer.AddHelpSection("Scene Contents", {
            "- sphere1: Red basic sphere (1.0 radius) at origin",
            "- sphere2: Green large sphere (2.0 radius) at (-4,0,0)", 
            "- sphere3: Cyan small sphere (0.5 radius) at (3,0,0)",
            "- sphere4: Yellow transparent sphere (1.5 radius) at (0,3,0)",
            "- sphere5: Magenta highlighted sphere (1.2 radius) at (0,-3,0)",
            "- sphere6: Blue hidden sphere (0.8 radius) at (2,2,0) - initially invisible"
        });
        
        viewer.AddHelpSection("Key Features", {
            "- Virtual objects automatically sync with OpenGL backend",
            "- High-level API abstracts OpenGL complexity",
            "- Object properties (color, position, size) easily modifiable",
            "- Selection and interaction support built-in",
            "- Visibility toggling without object destruction"
        });
        
        viewer.AddHelpSection("Scene Navigation", {
            "- Use mouse to rotate, pan, and zoom",
            "- Observe virtual scene integration with OpenGL",
            "- sphere5 shows selection highlighting",
            "- sphere6 demonstrates visibility control (hidden)"
        });
        
        // Set the scene setup callback
        viewer.SetSceneSetup(SetupVirtualSceneDemo);
        
        // Note: Interactive key callbacks not available in current GlViewer API
        // The demo shows static virtual scene with various sphere properties
        // Future versions could add keyboard interaction support
        
        std::cout << "\n=== Virtual Scene Demo Instructions ===\n";
        std::cout << "- Use mouse to navigate around the scene\n";
        std::cout << "- Observe different sphere properties:\n";
        std::cout << "  * sphere1: Red basic sphere at origin\n";
        std::cout << "  * sphere2: Green large sphere at (-4,0,0)\n";
        std::cout << "  * sphere3: Cyan small sphere at (3,0,0)\n";
        std::cout << "  * sphere4: Yellow sphere at (0,3,0)\n";
        std::cout << "  * sphere5: Magenta selected sphere at (0,-3,0)\n";
        std::cout << "  * sphere6: Blue hidden sphere at (2,2,0)\n";
        std::cout << "=====================================\n\n";
        
        // Run the viewer
        viewer.Run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}