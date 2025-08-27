/*
 * test_virtual_scene_demo.cpp
 *
 * Created on: August 27, 2025
 * Description: Demonstration test for virtual scene integration workflow
 * 
 * This test demonstrates how the vscene module is intended to be used,
 * showing the complete workflow from object creation to interaction handling.
 * It serves as both a test and documentation of the expected API usage.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <glm/glm.hpp>

#include "imview/viewer.hpp"
#include "imview/box.hpp"

// NOTE: These includes represent the intended vscene API
// They may not compile initially - that's expected for this design phase
#include "vscene/virtual_scene_panel.hpp"
#include "vscene/virtual_sphere.hpp"
#include "vscene/gl_render_backend.hpp"
#include "vscene/event_system.hpp"

using namespace quickviz;

/**
 * @brief Demonstration of intended vscene workflow
 * 
 * This test shows how applications should interact with the virtual scene system:
 * 1. Create virtual scene panel (replaces SceneViewPanel)
 * 2. Set up render backend (GlDrawBackend wraps existing gldraw)
 * 3. Create virtual objects with application semantics
 * 4. Subscribe to events for application logic
 * 5. Run interactive loop
 */
int main() {
    std::cout << "=== Virtual Scene Integration Demo ===" << std::endl;
    std::cout << "This demo shows the intended vscene workflow and API usage." << std::endl;

    try {
        // 1. Create viewer and layout (same as current system)
        Viewer viewer("Virtual Scene Demo", 1200, 800);
        
        auto main_box = std::make_shared<Box>("main_box");
        main_box->SetFlexDirection(Styling::FlexDirection::kRow);
        main_box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
        main_box->SetAlignItems(Styling::AlignItems::kStretch);

        // 2. Create virtual scene panel (evolution of SceneViewPanel)
        auto scene_panel = std::make_shared<VirtualScenePanel>("Virtual Scene");
        scene_panel->SetAutoLayout(true);
        scene_panel->SetNoTitleBar(true);
        scene_panel->SetFlexGrow(1.0f);
        scene_panel->SetFlexShrink(0.0f);

        // 3. Set up render backend (wraps existing gldraw system)
        auto render_backend = std::make_unique<GlRenderBackend>();
        scene_panel->SetRenderBackend(std::move(render_backend));

        // 4. Create virtual objects with application semantics
        std::cout << "Creating virtual objects..." << std::endl;

        // Sphere representing a waypoint
        auto waypoint1 = CreateVirtualSphere("waypoint_001", 0.5f);
        waypoint1->SetPosition(glm::vec3(-2.0f, 0.0f, 0.0f));
        waypoint1->SetColor(glm::vec3(0.0f, 1.0f, 0.0f)); // Green waypoint
        
        // Sphere representing a target
        auto target1 = CreateVirtualSphere("target_001", 0.3f);
        target1->SetPosition(glm::vec3(2.0f, 1.0f, 0.0f));
        target1->SetColor(glm::vec3(1.0f, 0.0f, 0.0f)); // Red target
        
        // Sphere representing an obstacle
        auto obstacle1 = CreateVirtualSphere("obstacle_001", 0.8f);
        obstacle1->SetPosition(glm::vec3(0.0f, -1.5f, 0.0f));
        obstacle1->SetColor(glm::vec3(0.5f, 0.5f, 0.5f)); // Gray obstacle

        // 5. Set up object-specific event callbacks
        waypoint1->OnClick = [](VirtualObject* obj, glm::vec2 screen_pos, glm::vec3 world_pos) {
            std::cout << "Waypoint " << obj->GetId() << " clicked at world position: "
                      << "(" << world_pos.x << ", " << world_pos.y << ", " << world_pos.z << ")" << std::endl;
        };

        waypoint1->OnDrag = [](VirtualObject* obj, glm::vec3 world_delta) {
            std::cout << "Waypoint " << obj->GetId() << " dragged by delta: "
                      << "(" << world_delta.x << ", " << world_delta.y << ", " << world_delta.z << ")" << std::endl;
        };

        target1->OnClick = [](VirtualObject* obj, glm::vec2 screen_pos, glm::vec3 world_pos) {
            std::cout << "Target " << obj->GetId() << " clicked! Showing target properties..." << std::endl;
        };

        obstacle1->OnHover = [](VirtualObject* obj, bool entering) {
            if (entering) {
                std::cout << "Mouse entered obstacle " << obj->GetId() << std::endl;
                obj->SetColor(glm::vec3(1.0f, 0.5f, 0.0f)); // Orange on hover
            } else {
                std::cout << "Mouse exited obstacle " << obj->GetId() << std::endl;
                obj->SetColor(glm::vec3(0.5f, 0.5f, 0.5f)); // Back to gray
            }
        };

        // 6. Add objects to scene
        scene_panel->AddObject("waypoint_001", std::move(waypoint1));
        scene_panel->AddObject("target_001", std::move(target1));
        scene_panel->AddObject("obstacle_001", std::move(obstacle1));

        // 7. Subscribe to global scene events for application logic
        auto event_dispatcher = scene_panel->GetEventDispatcher();
        
        // Handle background clicks (e.g., create new waypoints)
        event_dispatcher->Subscribe(VirtualEventType::BackgroundClicked,
            [scene_panel](const VirtualEvent& e) {
                if (e.ctrl_pressed) {
                    std::cout << "Ctrl+Click on background - would create new waypoint at: "
                              << "(" << e.world_pos.x << ", " << e.world_pos.y << ", " << e.world_pos.z << ")" << std::endl;
                    
                    // Example: Create new waypoint
                    static int waypoint_counter = 2;
                    std::string new_id = "waypoint_" + std::to_string(waypoint_counter++);
                    
                    auto new_waypoint = CreateVirtualSphere(new_id, 0.4f);
                    new_waypoint->SetPosition(e.world_pos);
                    new_waypoint->SetColor(glm::vec3(0.0f, 0.8f, 1.0f)); // Light blue
                    
                    new_waypoint->OnClick = [](VirtualObject* obj, glm::vec2, glm::vec3 world_pos) {
                        std::cout << "New waypoint " << obj->GetId() << " clicked!" << std::endl;
                    };
                    
                    scene_panel->AddObject(new_id, std::move(new_waypoint));
                }
            });

        // Handle selection changes (e.g., update property panel)
        event_dispatcher->Subscribe(VirtualEventType::SelectionChanged,
            [](const VirtualEvent& e) {
                if (e.object_ids.empty()) {
                    std::cout << "Selection cleared" << std::endl;
                } else {
                    std::cout << "Selection changed: ";
                    for (const auto& id : e.object_ids) {
                        std::cout << id << " ";
                    }
                    std::cout << std::endl;
                }
            });

        // 8. Set up layout and run
        main_box->AddChild(scene_panel);
        viewer.AddSceneObject(main_box);

        std::cout << "\n=== Interaction Instructions ===" << std::endl;
        std::cout << "Left Click: Select objects" << std::endl;
        std::cout << "Ctrl + Left Click (background): Create new waypoint" << std::endl;
        std::cout << "Mouse over obstacles: Hover effects" << std::endl;
        std::cout << "Drag objects: Move waypoints" << std::endl;
        std::cout << "ESC: Exit" << std::endl;
        std::cout << "=================================" << std::endl;

        // Note: In the real implementation, this would show the scene
        // For now, this demonstrates the intended API workflow
        std::cout << "\nAPI demonstration complete!" << std::endl;
        std::cout << "Virtual scene created with " 
                  << scene_panel->GetVirtualScene()->GetObjectIds().size() 
                  << " objects" << std::endl;

        // viewer.Show(); // Would run the interactive loop
        
        std::cout << "\nThis test demonstrates the intended vscene integration workflow." << std::endl;
        std::cout << "Key benefits shown:" << std::endl;
        std::cout << "1. Clean application-level object semantics (waypoints, targets, obstacles)" << std::endl;
        std::cout << "2. Event-driven interaction model" << std::endl;
        std::cout << "3. Separation of application logic from rendering" << std::endl;
        std::cout << "4. Easy object creation and manipulation" << std::endl;
        std::cout << "5. Extensible backend system" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Demo error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

/*
 * Expected Output (when interfaces are implemented):
 * 
 * === Virtual Scene Integration Demo ===
 * This demo shows the intended vscene workflow and API usage.
 * Creating virtual objects...
 * 
 * === Interaction Instructions ===
 * Left Click: Select objects
 * Ctrl + Left Click (background): Create new waypoint
 * Mouse over obstacles: Hover effects
 * Drag objects: Move waypoints
 * ESC: Exit
 * =================================
 * 
 * API demonstration complete!
 * Virtual scene created with 3 objects
 * 
 * This test demonstrates the intended vscene integration workflow.
 * Key benefits shown:
 * 1. Clean application-level object semantics (waypoints, targets, obstacles)
 * 2. Event-driven interaction model
 * 3. Separation of application logic from rendering
 * 4. Easy object creation and manipulation
 * 5. Extensible backend system
 * 
 * [Interactive mode would then show the 3D scene with objects]
 */