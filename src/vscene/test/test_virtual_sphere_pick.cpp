/*
 * @file test_virtual_sphere_pick.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date August 27, 2025
 * @brief Interactive virtual sphere picking test
 *
 * This test demonstrates real-time user picking and interaction with
 * VirtualSphere objects:
 * - Real mouse clicking for object selection
 * - Visual highlighting of picked/selected objects
 * - Interactive callbacks responding to user input
 * - VirtualScenePanel integration for proper mouse handling
 * - Selection state management with visual feedback
 *
 * Users can directly click on spheres to interact with them, and picked
 * objects are highlighted to provide immediate visual feedback.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <vector>
#include <iomanip>
#include <chrono>
#include <thread>

#include "imview/viewer.hpp"
#include "gldraw/gl_scene_panel.hpp"
#include "../../gldraw/include/gldraw/details/selection_manager.hpp"
#include "gldraw/renderable/grid.hpp"
#include "vscene/virtual_scene.hpp"
#include "vscene/virtual_sphere.hpp"
#include "vscene/gl_render_backend.hpp"

using namespace quickviz;

class VirtualSpherePickingDemo {
 public:
  VirtualSpherePickingDemo() = default;

  std::shared_ptr<GlScenePanel> CreateScenePanel() {
    std::cout << "\n=== Setting up Mouse & Keyboard Interactive Picking ===\n";

    // Create scene panel
    scene_panel_ =
        std::make_shared<GlScenePanel>("Interactive Sphere Picking");
    scene_panel_->SetAutoLayout(true);
    scene_panel_->SetNoTitleBar(true);
    scene_panel_->SetFlexGrow(1.0f);

    // Get the scene manager from the panel
    auto* gl_scene_manager = scene_panel_->GetSceneManager();

    // Create virtual scene with GlRenderBackend
    scene_ = std::make_unique<VirtualScene>();
    auto gl_backend = std::make_unique<GlRenderBackend>(gl_scene_manager);
    gl_backend_ = gl_backend.get();
    scene_->SetRenderBackend(std::move(gl_backend));

    CreateInteractiveSpheres();
    SetupEventSystem();
    SetupInputHandling();

    // Add reference grid
    AddReferenceGrid();

    // Initial scene update
    scene_->Update(0.0f);

    // All objects registered successfully
    auto object_ids = scene_->GetObjectIds();

    std::cout << "🖱️  Mouse clicking enabled - click on spheres!\n";
    std::cout << "⌨️  Keyboard shortcuts enabled - see help for details\n";
    std::cout << "✨ " << scene_->GetObjectIds().size()
              << " interactive spheres created\n\n";

    return scene_panel_;
  }

  void SetupInputHandling() {
    std::cout << "Setting up mouse and keyboard input handling:\n";

    // Set up new SelectionManager callback for mouse clicks
    scene_panel_->GetSelection().SetSelectionCallback(
        [this](const SelectionResult& result, const MultiSelection& multi) {
          if (std::holds_alternative<ObjectSelection>(result)) {
            auto obj_selection = std::get<ObjectSelection>(result);
            HandleObjectSelection(obj_selection.object_name);
          } else if (IsEmpty(result)) {
            HandleObjectSelection("");
          }
        });

    std::cout << "- Mouse click object selection enabled\n";
    std::cout << "- Object highlighting enabled\n";
    std::cout << "- Keyboard shortcuts registered\n\n";
  }

  void AddReferenceGrid() {
    std::cout << "Adding reference grid for spatial context:\n";
    
    // Get the scene manager to add OpenGL objects directly
    auto* gl_scene_manager = scene_panel_->GetSceneManager();
    
    // Create a grid with 10-unit size and 1-unit spacing
    auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.3f, 0.3f, 0.3f));
    
    // Add the grid to the scene manager
    gl_scene_manager->AddOpenGLObject("reference_grid", std::move(grid));
    
    std::cout << "- Added 10x10 reference grid with 1-unit spacing\n";
    std::cout << "- Grid color: Dark gray wireframe\n\n";
  }

  void ToggleGrid() {
    auto* gl_scene_manager = scene_panel_->GetSceneManager();
    
    grid_visible_ = !grid_visible_;
    
    if (grid_visible_) {
      // Re-add grid
      auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.3f, 0.3f, 0.3f));
      gl_scene_manager->AddOpenGLObject("reference_grid", std::move(grid));
      std::cout << "📐 Reference grid: VISIBLE" << std::endl;
    } else {
      // Remove grid
      gl_scene_manager->RemoveOpenGLObject("reference_grid");
      std::cout << "📐 Reference grid: HIDDEN" << std::endl;
    }
  }

  void HandleObjectSelection(const std::string& object_name) {
    total_clicks_++;

    if (object_name.empty()) {
      std::cout << "\n🖱️  Clicked background - no object selected\n";

      // Handle background clicks
      auto* dispatcher = scene_->GetEventDispatcher();
      if (dispatcher) {
        VirtualEvent event;
        event.type = VirtualEventType::BackgroundClicked;
        event.screen_pos = last_mouse_pos_;
        event.world_pos = glm::vec3(0.0f);  // Would need coordinate conversion
        dispatcher->Dispatch(event);
      }
      return;
    }

    std::cout << "\n🎯 PICKED OBJECT: '" << object_name << "'\n";

    // Find the virtual object and trigger its callback
    VirtualObject* picked = scene_->GetObject(object_name);
    if (picked && picked->OnClick) {
      // Highlight the selected object
      scene_->ClearSelection();
      scene_->AddToSelection(object_name);
      // TODO: Implement highlighting via new SelectionManager system
      // For now, selection is handled by the SelectionManager internally

      // Trigger the object's callback
      glm::vec2 screen_pos = last_mouse_pos_;
      glm::vec3 world_pos(0.0f);  // Simplified for demo
      picked->OnClick(picked, screen_pos, world_pos);

      successful_picks_++;

      // Update scene
      scene_->Update(0.0f);

      LogInteraction("USER_PICK", object_name, screen_pos, world_pos);
    }
  }

  void HandleKeyboardInput(int key) {
    std::cout << "\n⌨️  Keyboard key pressed: " << key << " (char: '"
              << (char)key << "')\n";

    switch (key) {
      case 'C':
      case 'c':
        ClearAllHighlights();
        break;
      case 'R':
      case 'r':
        ResetAllSpheres();
        break;
      case 'S':
      case 's':
        PrintStatistics();
        break;
      case 'G':
      case 'g':
        ToggleGrid();
        break;
      case '1':
        SelectSphereByIndex(0, "color_sphere");
        break;
      case '2':
        SelectSphereByIndex(1, "size_sphere");
        break;
      case '3':
        SelectSphereByIndex(2, "jump_sphere");
        break;
      case '4':
        SelectSphereByIndex(3, "select_sphere");
        break;
      case '5':
        SelectSphereByIndex(4, "info_sphere");
        break;
      default:
        std::cout << "Unknown key. Press H for help.\n";
        if (key == 'H' || key == 'h') {
          ShowKeyboardHelp();
        }
        break;
    }
  }

  void ClearAllHighlights() {
    std::cout << "🧹 Clearing all object highlights and selections\n";
    scene_->ClearSelection();

    // Clear highlights on all spheres
    std::vector<std::string> sphere_names = {"color_sphere", "size_sphere",
                                             "jump_sphere", "select_sphere",
                                             "info_sphere"};

    // TODO: Implement highlighting via new SelectionManager system
    // For now, clear all selections once (already done above with scene_->ClearSelection())
    scene_panel_->ClearSelection();

    scene_->Update(0.0f);
  }

  void ResetAllSpheres() {
    std::cout << "🔄 Resetting all spheres to initial state\n";

    // Reset each sphere to its original properties
    auto* color_sphere = scene_->GetObject("color_sphere");
    if (color_sphere) color_sphere->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));

    auto* size_sphere =
        dynamic_cast<VirtualSphere*>(scene_->GetObject("size_sphere"));
    if (size_sphere) size_sphere->SetRadius(0.5f);

    auto* jump_sphere = scene_->GetObject("jump_sphere");
    if (jump_sphere) jump_sphere->SetPosition(glm::vec3(3.0f, 0.0f, 0.0f));

    auto* select_sphere = scene_->GetObject("select_sphere");
    if (select_sphere) select_sphere->SetColor(glm::vec3(1.0f, 1.0f, 0.0f));

    ClearAllHighlights();
    scene_->Update(0.0f);
  }

  void SelectSphereByIndex(int index, const std::string& sphere_name) {
    std::cout << "🔢 Keyboard selection: " << sphere_name << " (key "
              << (index + 1) << ")\n";

    // Simulate clicking on the sphere
    HandleObjectSelection(sphere_name);
  }

  void ShowKeyboardHelp() {
    std::cout << "\n=== KEYBOARD SHORTCUTS ===\n";
    std::cout << "1-5: Select spheres directly\n";
    std::cout << "  1 - Red color sphere\n";
    std::cout << "  2 - Green size sphere\n";
    std::cout << "  3 - Blue jump sphere\n";
    std::cout << "  4 - Yellow select sphere\n";
    std::cout << "  5 - Cyan info sphere\n";
    std::cout << "C: Clear highlights\n";
    std::cout << "G: Toggle reference grid visibility\n";
    std::cout << "R: Reset all spheres\n";
    std::cout << "S: Show statistics\n";
    std::cout << "H: Show this help\n";
    std::cout << "========================\n\n";
  }

  void CreateInteractiveSpheres() {
    std::cout << "Creating interactive spheres with click behaviors:\n";

    // 1. Color-changing sphere (Red -> Rainbow cycle)
    auto colorSphere = std::make_unique<VirtualSphere>("color_sphere", 1.0f);
    colorSphere->SetPosition(glm::vec3(-2.0f, 0.0f, 0.0f));  // Left of center
    colorSphere->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));  // Start red
    colorSphere->OnClick = [this](VirtualObject* obj, glm::vec2 screen,
                                  glm::vec3 world) {
      // Highlight by selecting the object
      scene_->ClearSelection();
      scene_->AddToSelection(obj->GetId());

      static int color_index = 0;
      glm::vec3 colors[] = {
          glm::vec3(1.0f, 0.0f, 0.0f),  // Red
          glm::vec3(1.0f, 0.5f, 0.0f),  // Orange
          glm::vec3(1.0f, 1.0f, 0.0f),  // Yellow
          glm::vec3(0.0f, 1.0f, 0.0f),  // Green
          glm::vec3(0.0f, 0.0f, 1.0f),  // Blue
          glm::vec3(0.5f, 0.0f, 1.0f),  // Purple
          glm::vec3(1.0f, 0.0f, 1.0f)   // Magenta
      };
      color_index = (color_index + 1) % 7;
      obj->SetColor(colors[color_index]);
      std::cout << "🔴 Color sphere: Changed to color " << color_index
                << " (HIGHLIGHTED)\n";
      LogInteraction("COLOR_CHANGE", obj->GetId(), screen, world);
    };
    colorSphere->OnHover = [this](VirtualObject* obj, bool entering) {
      if (entering) {
        std::cout << "Hovering over color sphere (click to change color)\n";
      }
    };
    scene_->AddObject("color_sphere", std::move(colorSphere));

    // 2. Size-changing sphere (Small -> Large cycle)
    auto sizeSphere = std::make_unique<VirtualSphere>("size_sphere", 0.5f);
    sizeSphere->SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
    sizeSphere->SetColor(glm::vec3(0.0f, 1.0f, 0.0f));  // Green
    sizeSphere->OnClick = [this](VirtualObject* obj, glm::vec2 screen,
                                 glm::vec3 world) {
      // Highlight by selecting the object
      scene_->ClearSelection();
      scene_->AddToSelection(obj->GetId());

      auto* sphere = dynamic_cast<VirtualSphere*>(obj);
      if (sphere) {
        static float sizes[] = {0.5f, 1.0f, 1.5f, 2.0f};
        static int size_index = 0;
        size_index = (size_index + 1) % 4;
        sphere->SetRadius(sizes[size_index]);
        std::cout << "🟢 Size sphere: Changed radius to " << sizes[size_index]
                  << " (HIGHLIGHTED)\n";
        LogInteraction("SIZE_CHANGE", obj->GetId(), screen, world);
      }
    };
    sizeSphere->OnHover = [this](VirtualObject* obj, bool entering) {
      if (entering) {
        std::cout << "Hovering over size sphere (click to resize)\n";
      }
    };
    scene_->AddObject("size_sphere", std::move(sizeSphere));

    // 3. Position-jumping sphere (Teleports randomly)
    auto jumpSphere = std::make_unique<VirtualSphere>("jump_sphere", 0.8f);
    jumpSphere->SetPosition(glm::vec3(2.0f, 0.0f, 0.0f));  // Right of center
    jumpSphere->SetColor(glm::vec3(0.0f, 0.0f, 1.0f));  // Blue
    jumpSphere->OnClick = [this](VirtualObject* obj, glm::vec2 screen,
                                 glm::vec3 world) {
      // Highlight by selecting the object
      scene_->ClearSelection();
      scene_->AddToSelection(obj->GetId());

      // Random position within a constrained area to avoid interference with other spheres
      float x = 1.5f + (rand() % 100) / 100.0f;  // 1.5 to 2.5 (right side)
      float y = (rand() % 200 - 100) / 100.0f;  // -1 to 1
      float z = (rand() % 200 - 100) / 100.0f;  // -1 to 1
      glm::vec3 new_pos(x, y, z);
      obj->SetPosition(new_pos);
      std::cout << "🔵 Jump sphere: Teleported to (" << x << ", " << y << ", " << z << ") (HIGHLIGHTED)\n";
      LogInteraction("TELEPORT", obj->GetId(), screen, world);
    };
    jumpSphere->OnHover = [this](VirtualObject* obj, bool entering) {
      if (entering) {
        std::cout << "Hovering over jump sphere (click to teleport)\n";
      }
    };
    scene_->AddObject("jump_sphere", std::move(jumpSphere));

    // 4. Selection tracking sphere (Shows selection state)
    auto selectSphere = std::make_unique<VirtualSphere>("select_sphere", 1.2f);
    selectSphere->SetPosition(glm::vec3(0.0f, 2.0f, 0.0f));  // Above center
    selectSphere->SetColor(glm::vec3(1.0f, 1.0f, 0.0f));  // Yellow
    selectSphere->OnClick = [this](VirtualObject* obj, glm::vec2 screen,
                                   glm::vec3 world) {
      // Simplified approach: just toggle between yellow and magenta like key 5
      static bool toggle_state = false;
      toggle_state = !toggle_state;

      if (toggle_state) {
        obj->SetColor(glm::vec3(1.0f, 0.0f, 1.0f));  // Magenta
        std::cout << "✅ 🟡 KEY 4 WORKING: Changed to MAGENTA!\n";
        std::cout << "   Sphere should now be magenta in the scene.\n";

        // Also set selection for highlighting
        scene_->ClearSelection();
        scene_->AddToSelection(obj->GetId());
      } else {
        obj->SetColor(glm::vec3(1.0f, 1.0f, 0.0f));  // Yellow
        std::cout << "⬅️  🟡 KEY 4 WORKING: Changed to YELLOW!\n";
        std::cout << "   Sphere should now be yellow in the scene.\n";

        // Clear selection
        scene_->ClearSelection();
      }

      // Force backend update to propagate changes to OpenGL objects
      obj->UpdateBackend(gl_backend_);

      // Force scene update to propagate changes to render backend
      scene_->Update(0.0f);

      LogInteraction("SELECTION", obj->GetId(), screen, world);
    };
    selectSphere->OnHover = [this](VirtualObject* obj, bool entering) {
      if (entering) {
        std::cout
            << "Hovering over selection sphere (click to toggle selection)\n";
      }
    };
    scene_->AddObject("select_sphere", std::move(selectSphere));

    // 5. Information sphere (Shows interaction data)
    auto infoSphere = std::make_unique<VirtualSphere>("info_sphere", 1.0f);
    infoSphere->SetPosition(glm::vec3(0.0f, -2.0f, 0.0f));  // Below center
    infoSphere->SetColor(glm::vec3(0.0f, 1.0f, 1.0f));  // Cyan
    infoSphere->OnClick = [this](VirtualObject* obj, glm::vec2 screen,
                                 glm::vec3 world) {
      // Highlight by selecting the object
      scene_->ClearSelection();
      scene_->AddToSelection(obj->GetId());

      // Make visual change more obvious - flash between cyan and white
      static bool flash_state = false;
      if (flash_state) {
        obj->SetColor(glm::vec3(1.0f, 1.0f, 1.0f));  // White flash
      } else {
        obj->SetColor(glm::vec3(0.0f, 1.0f, 1.0f));  // Original cyan
      }
      flash_state = !flash_state;

      std::cout
          << "✅ 🔵 INFO SPHERE ACTIVATED: Showing detailed information!\n";
      std::cout << "   Key 5 worked! Info sphere response:\n";
      PrintDetailedInfo(obj, screen, world);
      std::cout << "   ☝️  Key 5 successfully triggered info display!\n";

      // Force backend update to propagate changes to OpenGL objects
      obj->UpdateBackend(gl_backend_);

      // Force scene update to propagate changes to render backend
      scene_->Update(0.0f);

      LogInteraction("INFO_DISPLAY", obj->GetId(), screen, world);
    };
    infoSphere->OnHover = [this](VirtualObject* obj, bool entering) {
      if (entering) {
        std::cout
            << "Hovering over info sphere (click for detailed information)\n";
        hover_count_++;
      } else {
        std::cout << "Stopped hovering over info sphere\n";
      }
    };
    scene_->AddObject("info_sphere", std::move(infoSphere));

    std::cout << "Created 5 interactive spheres with different behaviors:\n";
    std::cout << "- color_sphere (red): Changes color on click\n";
    std::cout << "- size_sphere (green): Changes size on click\n";
    std::cout << "- jump_sphere (blue): Teleports on click\n";
    std::cout << "- select_sphere (yellow): Shows selection state\n";
    std::cout << "- info_sphere (cyan): Displays detailed info\n\n";
  }

  void SetupEventSystem() {
    std::cout << "Setting up event system for scene-level interactions:\n";

    auto* dispatcher = scene_->GetEventDispatcher();
    if (!dispatcher) {
      std::cout << "Warning: Event dispatcher not available\n";
      return;
    }

    // Handle background clicks
    dispatcher->Subscribe(
        VirtualEventType::BackgroundClicked, [this](const VirtualEvent& e) {
          background_clicks_++;
          std::cout << "\nBackground clicked at world pos: (" << std::fixed
                    << std::setprecision(2) << e.world_pos.x << ", "
                    << e.world_pos.y << ", " << e.world_pos.z << ")\n";

          if (e.ctrl_pressed) {
            CreateSphereAtPosition(e.world_pos);
          } else if (e.shift_pressed) {
            scene_->ClearSelection();
            std::cout << "Cleared all selections (Shift+Click)\n";
          }

          LogInteraction("BACKGROUND_CLICK", "background",
                         glm::vec2(e.screen_pos), e.world_pos);
        });

    // Handle selection changes
    dispatcher->Subscribe(
        VirtualEventType::SelectionChanged, [this](const VirtualEvent& e) {
          auto selected_ids = scene_->GetSelectedIds();
          std::cout << "\nSelection changed - Currently selected: ";
          if (selected_ids.empty()) {
            std::cout << "None\n";
          } else {
            for (const auto& id : selected_ids) {
              std::cout << id << " ";
            }
            std::cout << "\n";
          }
          selection_changes_++;
        });

    std::cout << "Event handlers registered:\n";
    std::cout << "- Background clicks (Ctrl+Click to create sphere)\n";
    std::cout << "- Selection changes tracking\n";
    std::cout << "- Shift+Click to clear all selections\n\n";
  }

  void ShowInstructions() {
    std::cout << "\n=== INTERACTIVE PICKING INSTRUCTIONS ===\n";
    std::cout << "Click on any sphere to interact with it:\n\n";
    std::cout << "🔴 RED sphere (left): Changes color when clicked\n";
    std::cout << "🟢 GREEN sphere (center): Changes size when clicked\n";
    std::cout << "🔵 BLUE sphere (right): Teleports when clicked\n";
    std::cout << "🟡 YELLOW sphere (top): Toggles selection when clicked\n";
    std::cout << "🔵 CYAN sphere (bottom): Shows info when clicked\n\n";
    std::cout << "✨ Selected/picked objects will be highlighted!\n";
    std::cout << "📊 Statistics will update as you interact\n";
    std::cout << "=========================================\n\n";
  }

  void Update(float dt) {
    if (scene_) {
      scene_->Update(dt);
    }
  }

  void HandleKeyboardInput() {
    // This method can be used for ImGui-based input handling if needed
    // Currently, keyboard input is handled through GLFW callbacks
  }

  void PrintStatistics() {
    std::cout << "\n=== INTERACTION STATISTICS ===\n";
    std::cout << "Total clicks: " << total_clicks_ << "\n";
    std::cout << "Successful picks: " << successful_picks_ << "\n";
    std::cout << "Background clicks: " << background_clicks_ << "\n";
    std::cout << "Selection changes: " << selection_changes_ << "\n";
    std::cout << "Hover events: " << hover_count_ << "\n";
    std::cout << "Created spheres: " << created_spheres_ << "\n";
    std::cout << "Interaction logs: " << interaction_log_.size() << "\n";

    if (total_clicks_ > 0) {
      float pick_rate = (float)successful_picks_ / total_clicks_ * 100.0f;
      std::cout << "Pick success rate: " << std::fixed << std::setprecision(1)
                << pick_rate << "%\n";
    }
    std::cout << "==============================\n\n";
  }

 private:
  void CreateSphereAtPosition(const glm::vec3& world_pos) {
    std::string id = "created_sphere_" + std::to_string(created_spheres_);

    auto newSphere = std::make_unique<VirtualSphere>(id, 0.6f);
    newSphere->SetPosition(world_pos);

    // Random color for created spheres
    float r = (rand() % 100) / 100.0f;
    float g = (rand() % 100) / 100.0f;
    float b = (rand() % 100) / 100.0f;
    newSphere->SetColor(glm::vec3(r, g, b));

    // Add click callback to remove itself
    newSphere->OnClick = [this](VirtualObject* obj, glm::vec2 screen,
                                glm::vec3 world) {
      std::cout << "Created sphere " << obj->GetId() << " removing itself!\n";
      scene_->RemoveObject(obj->GetId());
    };

    newSphere->OnHover = [](VirtualObject* obj, bool entering) {
      if (entering) {
        std::cout << "Hovering over created sphere (click to remove)\n";
      }
    };

    scene_->AddObject(id, std::move(newSphere));
    created_spheres_++;

    std::cout << "Created new sphere '" << id << "' at world position ("
              << world_pos.x << ", " << world_pos.y << ", " << world_pos.z
              << ")\n";
  }

  void PrintDetailedInfo(VirtualObject* obj, glm::vec2 screen,
                         glm::vec3 world) {
    std::cout << "\n=== DETAILED OBJECT INFORMATION ===\n";
    std::cout << "Object ID: " << obj->GetId() << "\n";

    if (auto* sphere = dynamic_cast<VirtualSphere*>(obj)) {
      std::cout << "Type: VirtualSphere\n";
      std::cout << "Radius: " << sphere->GetRadius() << "\n";
      std::cout << "Tessellation: " << sphere->GetTessellation() << "\n";
    }

    std::cout << "Selected: "
              << (scene_->IsSelected(obj->GetId()) ? "YES" : "NO") << "\n";
    std::cout << "Screen pos: (" << screen.x << ", " << screen.y << ")\n";
    std::cout << "World pos: (" << world.x << ", " << world.y << ", " << world.z
              << ")\n";

    // Show recent interaction history for this object
    std::cout << "Recent interactions:\n";
    int count = 0;
    for (auto it = interaction_log_.rbegin();
         it != interaction_log_.rend() && count < 3; ++it) {
      if (it->object_id == obj->GetId()) {
        std::cout << "  " << it->action << " at (" << it->world_pos.x << ", "
                  << it->world_pos.y << ", " << it->world_pos.z << ")\n";
        count++;
      }
    }

    std::cout << "====================================\n\n";
  }

  void LogInteraction(const std::string& action, const std::string& object_id,
                      const glm::vec2& screen_pos, const glm::vec3& world_pos) {
    InteractionLog log;
    log.action = action;
    log.object_id = object_id;
    log.screen_pos = screen_pos;
    log.world_pos = world_pos;
    log.timestamp = std::chrono::steady_clock::now();

    interaction_log_.push_back(log);

    // Keep only last 50 interactions
    if (interaction_log_.size() > 50) {
      interaction_log_.erase(interaction_log_.begin());
    }
  }

  struct InteractionLog {
    std::string action;
    std::string object_id;
    glm::vec2 screen_pos;
    glm::vec3 world_pos;
    std::chrono::steady_clock::time_point timestamp;
  };

  std::shared_ptr<GlScenePanel> scene_panel_;
  std::unique_ptr<VirtualScene> scene_;
  GlRenderBackend* gl_backend_ = nullptr;

  // Mouse tracking
  glm::vec2 last_mouse_pos_{0.0f};

  // Statistics
  int total_clicks_ = 0;
  int successful_picks_ = 0;
  int background_clicks_ = 0;
  int selection_changes_ = 0;
  int hover_count_ = 0;
  int created_spheres_ = 0;

  // UI state
  bool grid_visible_ = true;

  std::vector<InteractionLog> interaction_log_;
};

// Global demo instance
std::unique_ptr<VirtualSpherePickingDemo> g_demo;

int main(int argc, char* argv[]) {
  try {
    // Create the demo instance
    g_demo = std::make_unique<VirtualSpherePickingDemo>();

    // Create viewer for mouse and keyboard interaction
    Viewer viewer("Interactive Virtual Sphere Picking - Mouse & Keyboard", 1200,
                  800);

    // Create the interactive scene panel
    auto scene_panel = g_demo->CreateScenePanel();

    // Enable keyboard navigation for input handling
    viewer.EnableKeyboardNav(true);

    // Add the scene panel to the viewer
    viewer.AddSceneObject(scene_panel);

    // Set up update callback for keyboard input
    scene_panel->SetPreDrawCallback([&]() {
      g_demo->Update(0.016f);  // ~60 FPS
    });

    // Set up GLFW keyboard callback for direct keyboard input
    glfwSetKeyCallback(viewer.GetWindowObject(), [](GLFWwindow* window, int key, int scancode, int action, int mods) {
      if (action == GLFW_PRESS && g_demo) {
        g_demo->HandleKeyboardInput(key);
      }
    });

    // Show initial instructions
    std::cout << "\n=== 🖱️ ⌨️  MOUSE & KEYBOARD INTERACTIVE DEMO ===\n";
    std::cout << "🖱️  MOUSE CONTROLS:\n";
    std::cout << "   - Left click on any colored sphere to interact\n";
    std::cout << "   - Clicked spheres will be highlighted automatically\n";
    std::cout << "   - Background clicks are detected\n\n";
    std::cout << "⌨️  KEYBOARD SHORTCUTS:\n";
    std::cout << "   - 1-5: Select spheres directly (1=red, 2=green, 3=blue, "
                 "4=yellow, 5=cyan)\n";
    std::cout << "   - C: Clear highlights\n";
    std::cout << "   - G: Toggle reference grid\n";
    std::cout << "   - R: Reset all spheres to initial state\n";
    std::cout << "   - S: Show statistics\n";
    std::cout << "   - H: Show keyboard help\n\n";
    std::cout << "✨ SPHERE BEHAVIORS:\n";
    std::cout << "   🔴 RED: Changes color when clicked\n";
    std::cout << "   🟢 GREEN: Changes size when clicked\n";
    std::cout << "   🔵 BLUE: Teleports when clicked\n";
    std::cout << "   🟡 YELLOW: Toggles selection state\n";
    std::cout << "   🔵 CYAN: Shows detailed information\n";
    std::cout << "================================================\n\n";

    // Show the interactive viewer
    viewer.Show();

    // Important: Clean up demo before viewer is destroyed to prevent segfault
    // The demo contains OpenGL resources that must be cleaned up while context
    // is still valid
    g_demo.reset();

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    g_demo.reset();  // Clean up on exception too
    return 1;
  }

  return 0;
}