/*
 * @file test_vector_field.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-24
 * @brief Test for VectorField rendering functionality
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include <functional>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "gldraw/gl_view.hpp"
#include "gldraw/renderable/vector_field.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/coordinate_frame.hpp"

using namespace quickviz;

void SetupVectorFieldScene(GlSceneManager* scene_manager) {
    // Add grid for reference
    auto grid = std::make_unique<Grid>(12.0f, 1.0f, glm::vec3(0.3f, 0.3f, 0.3f));
    scene_manager->AddOpenGLObject("grid", std::move(grid));
    
    // Add coordinate frame for reference
    auto frame = std::make_unique<CoordinateFrame>(2.0f);
    scene_manager->AddOpenGLObject("frame", std::move(frame));
    
    // 1. Simple radial vector field - arrows mode
    auto field1 = std::make_unique<VectorField>();
    std::vector<glm::vec3> origins1, vectors1;
    for (int i = -2; i <= 2; ++i) {
        for (int j = -2; j <= 2; ++j) {
            glm::vec3 pos(i * 1.0f - 4.0f, j * 1.0f + 3.0f, 0.5f);
            glm::vec3 center(-4.0f, 3.0f, 0.5f);
            glm::vec3 vec = glm::normalize(pos - center) * 0.8f;
            if (glm::length(pos - center) > 0.1f) {
                origins1.push_back(pos);
                vectors1.push_back(vec);
            }
        }
    }
    field1->SetVectors(origins1, vectors1);
    field1->SetRenderStyle(VectorField::RenderStyle::kArrows3D);
    field1->SetColorMode(VectorField::ColorMode::kUniform);
    field1->SetUniformColor(glm::vec3(1.0f, 0.2f, 0.2f));  // Red
    field1->SetArrowScale(1.0f);
    scene_manager->AddOpenGLObject("field_radial", std::move(field1));
    
    // 2. Spiral vector field - magnitude coloring
    auto field2 = std::make_unique<VectorField>();
    std::vector<glm::vec3> origins2, vectors2;
    for (int i = -3; i <= 3; ++i) {
        for (int j = -3; j <= 3; ++j) {
            glm::vec3 pos(i * 0.7f + 3.0f, j * 0.7f + 3.0f, 1.0f);
            float r = glm::length(glm::vec2(i, j)) * 0.7f;
            float angle = atan2(j, i);
            glm::vec3 tangent(-sin(angle), cos(angle), 0.0f);
            glm::vec3 radial(cos(angle), sin(angle), 0.0f);
            glm::vec3 vec = (tangent * 0.6f + radial * 0.3f) * (2.0f - r * 0.3f);
            if (glm::length(vec) > 0.1f) {
                origins2.push_back(pos);
                vectors2.push_back(vec);
            }
        }
    }
    field2->SetVectors(origins2, vectors2);
    field2->SetRenderStyle(VectorField::RenderStyle::kArrows3D);
    field2->SetColorMode(VectorField::ColorMode::kMagnitude);
    field2->SetColorRange(0.0f, 2.0f);
    field2->SetArrowScale(0.8f);
    scene_manager->AddOpenGLObject("field_spiral", std::move(field2));
    
    // 3. Gravitational field - lines mode with direction coloring
    auto field3 = std::make_unique<VectorField>();
    std::vector<glm::vec3> origins3, vectors3;
    glm::vec3 gravity_center(-3.0f, -2.0f, 1.5f);
    for (int i = -2; i <= 2; ++i) {
        for (int j = -2; j <= 2; ++j) {
            for (int k = 0; k <= 2; ++k) {
                glm::vec3 pos(i * 0.8f - 3.0f, j * 0.8f - 2.0f, k * 0.8f + 0.5f);
                if (glm::distance(pos, gravity_center) < 0.5f) continue;
                glm::vec3 to_center = gravity_center - pos;
                float dist_sq = glm::dot(to_center, to_center);
                glm::vec3 vec = glm::normalize(to_center) * (1.5f / dist_sq);
                if (glm::length(vec) > 0.1f && glm::length(vec) < 2.0f) {
                    origins3.push_back(pos);
                    vectors3.push_back(vec);
                }
            }
        }
    }
    field3->SetVectors(origins3, vectors3);
    field3->SetRenderStyle(VectorField::RenderStyle::kLines);
    field3->SetColorMode(VectorField::ColorMode::kDirection);
    field3->SetArrowScale(1.2f);
    field3->SetLineWidth(2.0f);
    scene_manager->AddOpenGLObject("field_gravity", std::move(field3));
    
    // 4. Wind field with custom colors
    auto field4 = std::make_unique<VectorField>();
    std::vector<glm::vec3> origins4, vectors4, colors4;
    for (int i = -2; i <= 2; ++i) {
        for (int j = -2; j <= 2; ++j) {
            glm::vec3 pos(i * 0.9f + 3.5f, j * 0.9f - 2.0f, 2.0f);
            // Create swirling wind pattern
            float x = i * 0.9f, y = j * 0.9f;
            glm::vec3 vec;
            vec.x = -y + 0.3f * sin(x * 2.0f);
            vec.y = x + 0.2f * cos(y * 1.5f);
            vec.z = 0.1f * sin(x + y);
            vec *= 0.7f;
            
            // Color based on height component
            glm::vec3 color;
            if (vec.z > 0) {
                color = glm::vec3(0.2f, 1.0f, 0.2f);  // Green for up
            } else {
                color = glm::vec3(0.2f, 0.2f, 1.0f);  // Blue for down
            }
            
            origins4.push_back(pos);
            vectors4.push_back(vec);
            colors4.push_back(color);
        }
    }
    field4->SetVectors(origins4, vectors4);
    field4->SetCustomColors(colors4);
    field4->SetRenderStyle(VectorField::RenderStyle::kArrows3D);
    field4->SetColorMode(VectorField::ColorMode::kCustom);
    field4->SetArrowScale(1.5f);
    scene_manager->AddOpenGLObject("field_wind", std::move(field4));
    
    // 5. Electromagnetic field - DISABLED (may implement curved field lines in future)
    /*
    auto field5 = std::make_unique<VectorField>();
    std::vector<glm::vec3> origins5, vectors5;
    // Create two "charges" - one positive, one negative (positioned away from other fields)
    glm::vec3 pos_charge(-6.0f, 0.0f, 3.5f);
    glm::vec3 neg_charge(-6.0f, -2.0f, 3.5f);
    
    for (int i = -3; i <= 3; ++i) {
        for (int j = -3; j <= 3; ++j) {
            glm::vec3 pos(i * 0.6f - 6.0f, j * 0.6f - 1.0f, 3.5f);
            // Electric field from two point charges
            glm::vec3 to_pos = pos - pos_charge;
            glm::vec3 to_neg = pos - neg_charge;
            float dist_pos = glm::length(to_pos);
            float dist_neg = glm::length(to_neg);
            
            if (dist_pos > 0.2f && dist_neg > 0.2f) {
                glm::vec3 field_pos = glm::normalize(to_pos) / (dist_pos * dist_pos);
                glm::vec3 field_neg = -glm::normalize(to_neg) / (dist_neg * dist_neg);
                glm::vec3 total_field = (field_pos + field_neg) * 0.5f;
                
                if (glm::length(total_field) > 0.1f && glm::length(total_field) < 3.0f) {
                    origins5.push_back(pos);
                    vectors5.push_back(total_field);
                }
            }
        }
    }
    field5->SetVectors(origins5, vectors5);
    field5->SetRenderStyle(VectorField::RenderStyle::kLines);
    field5->SetColorMode(VectorField::ColorMode::kMagnitude);
    field5->SetColorRange(0.0f, 1.5f);
    field5->SetLineWidth(2.0f);
    field5->SetOpacity(0.8f);
    scene_manager->AddOpenGLObject("field_electric", std::move(field5));
    */
    
    // 6. Mathematical gradient field - grid generation
    auto field6 = std::make_unique<VectorField>();
    field6->GenerateGridField(
        glm::vec3(-2.0f, -5.0f, 3.0f), 
        glm::vec3(2.0f, -1.0f, 4.0f), 
        glm::ivec3(8, 8, 2),
        [](const glm::vec3& pos) {
            // Saddle point field
            return glm::vec3(pos.x, -pos.y, (pos.x*pos.x - pos.y*pos.y) * 0.1f) * 0.3f;
        });
    field6->SetRenderStyle(VectorField::RenderStyle::kArrows3D);
    field6->SetColorMode(VectorField::ColorMode::kUniform);
    field6->SetUniformColor(glm::vec3(1.0f, 0.5f, 0.0f));  // Orange
    field6->SetArrowScale(1.0f);
    scene_manager->AddOpenGLObject("field_saddle", std::move(field6));
    
    // 7. Dense turbulent field with subsampling
    auto field7 = std::make_unique<VectorField>();
    std::vector<glm::vec3> origins7, vectors7;
    for (int i = -10; i <= 10; ++i) {
        for (int j = -10; j <= 10; ++j) {
            glm::vec3 pos(i * 0.2f + 4.0f, j * 0.2f - 3.0f, 3.5f);
            // Turbulent flow field
            float noise = sin(i * 0.5f) * cos(j * 0.3f);
            glm::vec3 vec(cos(i * 0.3f + j * 0.2f), sin(i * 0.2f - j * 0.4f), noise * 0.2f);
            vec *= (0.5f + noise * 0.3f);
            origins7.push_back(pos);
            vectors7.push_back(vec);
        }
    }
    field7->SetVectors(origins7, vectors7);
    field7->SetRenderStyle(VectorField::RenderStyle::kArrows3D);
    field7->SetColorMode(VectorField::ColorMode::kMagnitude);
    field7->SetColorRange(0.0f, 1.0f);
    field7->SetSubsampling(0.3f);  // Show only 30% of vectors
    field7->SetArrowScale(0.7f);
    field7->SetMagnitudeThreshold(0.1f);  // Hide very small vectors
    scene_manager->AddOpenGLObject("field_turbulent", std::move(field7));
    
    // 8. Vortex field with transparency
    auto field8 = std::make_unique<VectorField>();
    std::vector<glm::vec3> origins8, vectors8;
    glm::vec3 vortex_center(-3.0f, -3.0f, 4.5f);
    for (int i = -4; i <= 4; ++i) {
        for (int j = -4; j <= 4; ++j) {
            glm::vec3 pos(i * 0.5f - 3.0f, j * 0.5f - 3.0f, 4.5f);
            glm::vec3 to_center = pos - vortex_center;
            float dist = glm::length(to_center);
            if (dist > 0.1f && dist < 2.5f) {
                // Rotating field with radial component
                glm::vec3 tangent = glm::normalize(glm::vec3(-to_center.y, to_center.x, 0.0f));
                glm::vec3 radial = glm::normalize(glm::vec3(to_center.x, to_center.y, 0.0f));
                glm::vec3 vec = (tangent * 2.0f - radial * 0.3f) / (1.0f + dist * 0.5f);
                origins8.push_back(pos);
                vectors8.push_back(vec);
            }
        }
    }
    field8->SetVectors(origins8, vectors8);
    field8->SetRenderStyle(VectorField::RenderStyle::kArrows3D);
    field8->SetColorMode(VectorField::ColorMode::kUniform);
    field8->SetUniformColor(glm::vec3(0.8f, 0.2f, 0.8f));  // Magenta
    field8->SetArrowScale(0.8f);
    field8->SetOpacity(0.7f);
    scene_manager->AddOpenGLObject("field_vortex", std::move(field8));
}

int main(int argc, char* argv[]) {
    try {
        // Configure the view
        GlView::Config config;
        config.window_title = "Vector Field Rendering Test";
        config.coordinate_frame_size = 1.0f;
        
        // Create the view
        GlView view(config);
        
        // Set up description and help sections
        view.SetDescription("Testing vector field rendering for force fields, velocity fields, and gradients");
        
        view.AddHelpSection("Vector Field Features Demonstrated", {
            "- Multiple render styles: 3D arrows with lighting, lines",
            "- Color encoding: uniform, magnitude, direction, custom colors",
            "- Grid-based field generation from mathematical functions",
            "- Subsampling for dense fields and performance control",
            "- Transparency and opacity control",
            "- Magnitude thresholding to hide insignificant vectors"
        });
        
        view.AddHelpSection("Scene Contents", {
            "- Red radial field: Outward arrows from central point (uniform color)",
            "- Spiral field: Tangential + radial components (magnitude coloring)",
            "- Gravity field: Inverse-square attraction field (lines, direction colors)",
            "- Wind field: Swirling pattern with height component (custom colors)",
            "- Saddle field: Mathematical gradient field from grid generation (orange)",
            "- Turbulent field: Dense flow with subsampling (30% shown, magnitude colors)",
            "- Vortex field: Rotational field with transparency (magenta)"
        });
        
        view.AddHelpSection("Robotics Applications", {
            "- Force and potential field visualization for path planning",
            "- Velocity and acceleration field display",
            "- Wind and flow field representation for UAVs",
            "- Electromagnetic field visualization for sensor planning",
            "- Gradient field display for optimization algorithms",
            "- Motion planning vector fields and navigation functions"
        });
        
        // Set the scene setup callback
        view.SetSceneSetup(SetupVectorFieldScene);
        
        // Run the view
        view.Run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}