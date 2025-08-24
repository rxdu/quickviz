/*
 * @file test_uncertainty_ellipse.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-24
 * @brief Test for UncertaintyEllipse rendering functionality
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include <random>

#include "gldraw/gl_view.hpp"
#include "gldraw/renderable/uncertainty_ellipse.hpp"
#include "gldraw/renderable/sphere.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/coordinate_frame.hpp"
#include "gldraw/renderable/point_cloud.hpp"

using namespace quickviz;

// Forward declarations
void Create2DUncertaintyEllipses(GlSceneManager* scene_manager);
void Create3DUncertaintyEllipsoids(GlSceneManager* scene_manager);
void CreateCylindricalUncertainty(GlSceneManager* scene_manager);
void CreateMultiLevelConfidence(GlSceneManager* scene_manager);
void CreateCovarianceBasedEllipses(GlSceneManager* scene_manager);
void CreateSamplePoints(GlSceneManager* scene_manager);
void CreateReferenceObjects(GlSceneManager* scene_manager);

void SetupUncertaintyScene(GlSceneManager* scene_manager) {
    // Add reference objects
    CreateReferenceObjects(scene_manager);
    
    // 1. 2D uncertainty ellipses
    Create2DUncertaintyEllipses(scene_manager);
    
    // 2. 3D uncertainty ellipsoids
    Create3DUncertaintyEllipsoids(scene_manager);
    
    // 3. Cylindrical uncertainty volumes
    CreateCylindricalUncertainty(scene_manager);
    
    // 4. Multi-level confidence regions
    CreateMultiLevelConfidence(scene_manager);
    
    // 5. Covariance matrix based ellipses
    CreateCovarianceBasedEllipses(scene_manager);
    
    // 6. Sample points for visualization
    CreateSamplePoints(scene_manager);
}

void CreateReferenceObjects(GlSceneManager* scene_manager) {
    // Grid for reference
    auto grid = std::make_unique<Grid>(16.0f, 1.0f, glm::vec3(0.3f, 0.3f, 0.3f));
    scene_manager->AddOpenGLObject("grid", std::move(grid));
    
    // Coordinate frame
    auto frame = std::make_unique<CoordinateFrame>(1.5f);
    scene_manager->AddOpenGLObject("frame", std::move(frame));
}

void Create2DUncertaintyEllipses(GlSceneManager* scene_manager) {
    // 1. 1-sigma confidence ellipse (68.27%)
    auto ellipse_1s = std::make_unique<UncertaintyEllipse>(UncertaintyEllipse::EllipseType::k2D);
    ellipse_1s->SetCenter(glm::vec3(-4.0f, 3.0f, 0.0f));
    ellipse_1s->SetAxisLengths2D(1.5f, 0.8f, glm::radians(30.0f));
    ellipse_1s->SetConfidenceLevel(UncertaintyEllipse::ConfidenceLevel::kOneSigma);
    ellipse_1s->SetRenderMode(UncertaintyEllipse::RenderMode::kTransparent);
    ellipse_1s->SetColor(glm::vec3(0.2f, 0.8f, 0.2f));
    ellipse_1s->SetTransparency(0.4f);
    ellipse_1s->SetOutlineColor(glm::vec3(0.0f, 0.6f, 0.0f));
    ellipse_1s->SetRenderMode(UncertaintyEllipse::RenderMode::kOutlined);
    scene_manager->AddOpenGLObject("ellipse_1sigma", std::move(ellipse_1s));
    
    // Center point for reference
    auto center1 = std::make_unique<Sphere>(glm::vec3(-4.0f, 3.0f, 0.0f), 0.08f);
    center1->SetColor(glm::vec3(0.0f, 0.6f, 0.0f));
    scene_manager->AddOpenGLObject("center_1sigma", std::move(center1));
    
    // 2. 2-sigma confidence ellipse (95.45%)
    auto ellipse_2s = std::make_unique<UncertaintyEllipse>(UncertaintyEllipse::EllipseType::k2D);
    ellipse_2s->SetCenter(glm::vec3(0.0f, 4.0f, 0.0f));
    ellipse_2s->SetAxisLengths2D(2.0f, 1.2f, glm::radians(-15.0f));
    ellipse_2s->SetConfidenceLevel(UncertaintyEllipse::ConfidenceLevel::kTwoSigma);
    ellipse_2s->SetRenderMode(UncertaintyEllipse::RenderMode::kGradient);
    ellipse_2s->SetGradientColors(glm::vec3(1.0f, 0.8f, 0.0f), glm::vec3(1.0f, 0.4f, 0.0f));
    ellipse_2s->SetTransparency(0.5f);
    scene_manager->AddOpenGLObject("ellipse_2sigma", std::move(ellipse_2s));
    
    // Center point for reference
    auto center2 = std::make_unique<Sphere>(glm::vec3(0.0f, 4.0f, 0.0f), 0.08f);
    center2->SetColor(glm::vec3(1.0f, 0.6f, 0.0f));
    scene_manager->AddOpenGLObject("center_2sigma", std::move(center2));
    
    // 3. 3-sigma confidence ellipse (99.73%) - highly elongated
    auto ellipse_3s = std::make_unique<UncertaintyEllipse>(UncertaintyEllipse::EllipseType::k2D);
    ellipse_3s->SetCenter(glm::vec3(4.0f, 3.0f, 0.0f));
    ellipse_3s->SetAxisLengths2D(2.5f, 0.6f, glm::radians(60.0f));
    ellipse_3s->SetConfidenceLevel(UncertaintyEllipse::ConfidenceLevel::kThreeSigma);
    ellipse_3s->SetRenderMode(UncertaintyEllipse::RenderMode::kWireframe);
    ellipse_3s->SetOutlineColor(glm::vec3(0.8f, 0.0f, 0.8f));
    ellipse_3s->SetOutlineWidth(2.5f);
    scene_manager->AddOpenGLObject("ellipse_3sigma", std::move(ellipse_3s));
    
    // Center point for reference
    auto center3 = std::make_unique<Sphere>(glm::vec3(4.0f, 3.0f, 0.0f), 0.08f);
    center3->SetColor(glm::vec3(0.8f, 0.0f, 0.8f));
    scene_manager->AddOpenGLObject("center_3sigma", std::move(center3));
}

void Create3DUncertaintyEllipsoids(GlSceneManager* scene_manager) {
    // 1. Spherical uncertainty (equal axes)
    auto ellipsoid_sphere = std::make_unique<UncertaintyEllipse>(UncertaintyEllipse::EllipseType::k3D);
    ellipsoid_sphere->SetCenter(glm::vec3(-4.0f, 0.0f, 1.5f));
    ellipsoid_sphere->SetAxisLengths3D(glm::vec3(1.2f, 1.2f, 1.2f));
    ellipsoid_sphere->SetConfidenceLevel(UncertaintyEllipse::ConfidenceLevel::kOneSigma);
    ellipsoid_sphere->SetRenderMode(UncertaintyEllipse::RenderMode::kTransparent);
    ellipsoid_sphere->SetColor(glm::vec3(0.0f, 0.5f, 1.0f));
    ellipsoid_sphere->SetTransparency(0.3f);
    ellipsoid_sphere->SetResolution3D(16, 24);
    scene_manager->AddOpenGLObject("ellipsoid_sphere", std::move(ellipsoid_sphere));
    
    // 2. Oblate ellipsoid (flattened)
    auto ellipsoid_oblate = std::make_unique<UncertaintyEllipse>(UncertaintyEllipse::EllipseType::k3D);
    ellipsoid_oblate->SetCenter(glm::vec3(0.0f, 0.0f, 2.0f));
    ellipsoid_oblate->SetAxisLengths3D(glm::vec3(1.8f, 1.8f, 0.8f));
    ellipsoid_oblate->SetConfidenceLevel(UncertaintyEllipse::ConfidenceLevel::kTwoSigma);
    ellipsoid_oblate->SetRenderMode(UncertaintyEllipse::RenderMode::kFilled);
    ellipsoid_oblate->SetColor(glm::vec3(1.0f, 0.2f, 0.2f));
    ellipsoid_oblate->SetTransparency(0.4f);
    ellipsoid_oblate->SetResolution3D(12, 20);
    scene_manager->AddOpenGLObject("ellipsoid_oblate", std::move(ellipsoid_oblate));
    
    // 3. Prolate ellipsoid (elongated) with rotation
    auto ellipsoid_prolate = std::make_unique<UncertaintyEllipse>(UncertaintyEllipse::EllipseType::k3D);
    ellipsoid_prolate->SetCenter(glm::vec3(4.0f, 0.0f, 1.0f));
    
    // Create rotation matrix (45 degrees around Y axis)
    glm::mat3 rotation = glm::mat3(
        std::cos(glm::radians(45.0f)), 0.0f, std::sin(glm::radians(45.0f)),
        0.0f, 1.0f, 0.0f,
        -std::sin(glm::radians(45.0f)), 0.0f, std::cos(glm::radians(45.0f))
    );
    ellipsoid_prolate->SetAxisLengths3D(glm::vec3(0.8f, 1.0f, 2.2f), rotation);
    ellipsoid_prolate->SetConfidenceLevel(UncertaintyEllipse::ConfidenceLevel::kCustom);
    ellipsoid_prolate->SetSigmaMultiplier(1.5f);
    ellipsoid_prolate->SetRenderMode(UncertaintyEllipse::RenderMode::kOutlined);
    ellipsoid_prolate->SetColor(glm::vec3(0.8f, 0.8f, 0.2f));
    ellipsoid_prolate->SetOutlineColor(glm::vec3(0.6f, 0.6f, 0.0f));
    ellipsoid_prolate->SetTransparency(0.35f);
    scene_manager->AddOpenGLObject("ellipsoid_prolate", std::move(ellipsoid_prolate));
    
    // Center points for reference
    auto center_sphere = std::make_unique<Sphere>(glm::vec3(-4.0f, 0.0f, 1.5f), 0.08f);
    center_sphere->SetColor(glm::vec3(0.0f, 0.3f, 0.8f));
    scene_manager->AddOpenGLObject("center_sphere", std::move(center_sphere));
    
    auto center_oblate = std::make_unique<Sphere>(glm::vec3(0.0f, 0.0f, 2.0f), 0.08f);
    center_oblate->SetColor(glm::vec3(0.8f, 0.0f, 0.0f));
    scene_manager->AddOpenGLObject("center_oblate", std::move(center_oblate));
    
    auto center_prolate = std::make_unique<Sphere>(glm::vec3(4.0f, 0.0f, 1.0f), 0.08f);
    center_prolate->SetColor(glm::vec3(0.6f, 0.6f, 0.0f));
    scene_manager->AddOpenGLObject("center_prolate", std::move(center_prolate));
}

void CreateCylindricalUncertainty(GlSceneManager* scene_manager) {
    // Cylindrical uncertainty - useful for 2D localization with height uncertainty
    auto cylinder_unc = std::make_unique<UncertaintyEllipse>(UncertaintyEllipse::EllipseType::kCylindrical);
    cylinder_unc->SetCenter(glm::vec3(-2.0f, -3.0f, 1.0f));
    cylinder_unc->SetAxisLengths2D(1.5f, 0.9f, glm::radians(20.0f));
    cylinder_unc->SetCylindricalHeight(2.0f);
    cylinder_unc->SetConfidenceLevel(UncertaintyEllipse::ConfidenceLevel::kTwoSigma);
    cylinder_unc->SetRenderMode(UncertaintyEllipse::RenderMode::kTransparent);
    cylinder_unc->SetColor(glm::vec3(0.6f, 0.2f, 0.8f));
    cylinder_unc->SetOutlineColor(glm::vec3(0.4f, 0.0f, 0.6f));
    cylinder_unc->SetTransparency(0.25f);
    cylinder_unc->SetRenderMode(UncertaintyEllipse::RenderMode::kOutlined);
    scene_manager->AddOpenGLObject("cylinder_uncertainty", std::move(cylinder_unc));
    
    // Center reference
    auto center_cyl = std::make_unique<Sphere>(glm::vec3(-2.0f, -3.0f, 1.0f), 0.08f);
    center_cyl->SetColor(glm::vec3(0.4f, 0.0f, 0.6f));
    scene_manager->AddOpenGLObject("center_cylinder", std::move(center_cyl));
}

void CreateMultiLevelConfidence(GlSceneManager* scene_manager) {
    // Multi-level confidence visualization (nested ellipses)
    auto multi_level = std::make_unique<UncertaintyEllipse>(UncertaintyEllipse::EllipseType::k2D);
    multi_level->SetCenter(glm::vec3(2.0f, -3.0f, 0.0f));
    multi_level->SetAxisLengths2D(2.0f, 1.0f, glm::radians(-30.0f));
    multi_level->SetMultiLevel(true);
    
    // Add multiple confidence levels
    multi_level->AddConfidenceLevel(1.0f, glm::vec3(0.0f, 1.0f, 0.0f), 0.6f);    // 1-sigma (green)
    multi_level->AddConfidenceLevel(2.0f, glm::vec3(1.0f, 1.0f, 0.0f), 0.4f);    // 2-sigma (yellow)  
    multi_level->AddConfidenceLevel(3.0f, glm::vec3(1.0f, 0.0f, 0.0f), 0.2f);    // 3-sigma (red)
    
    multi_level->SetRenderMode(UncertaintyEllipse::RenderMode::kTransparent);
    scene_manager->AddOpenGLObject("multi_level_confidence", std::move(multi_level));
    
    // Center reference
    auto center_multi = std::make_unique<Sphere>(glm::vec3(2.0f, -3.0f, 0.0f), 0.08f);
    center_multi->SetColor(glm::vec3(0.0f, 0.0f, 0.0f));
    scene_manager->AddOpenGLObject("center_multi_level", std::move(center_multi));
}

void CreateCovarianceBasedEllipses(GlSceneManager* scene_manager) {
    // 1. Covariance matrix example 1 - diagonal covariance
    glm::mat2 cov1(
        2.0f, 0.0f,
        0.0f, 0.8f
    );
    
    auto ellipse_cov1 = std::make_unique<UncertaintyEllipse>(UncertaintyEllipse::EllipseType::k2D);
    ellipse_cov1->SetCenter(glm::vec3(-6.0f, -1.0f, 0.0f));
    ellipse_cov1->SetCovarianceMatrix2D(cov1);
    ellipse_cov1->SetConfidenceLevel(UncertaintyEllipse::ConfidenceLevel::kTwoSigma);
    ellipse_cov1->SetRenderMode(UncertaintyEllipse::RenderMode::kGradient);
    ellipse_cov1->SetGradientColors(glm::vec3(0.8f, 0.4f, 0.8f), glm::vec3(0.4f, 0.0f, 0.4f));
    ellipse_cov1->SetTransparency(0.5f);
    scene_manager->AddOpenGLObject("ellipse_cov_diagonal", std::move(ellipse_cov1));
    
    // 2. Covariance matrix example 2 - correlated covariance
    glm::mat2 cov2(
        1.5f, 0.8f,
        0.8f, 1.2f
    );
    
    auto ellipse_cov2 = std::make_unique<UncertaintyEllipse>(UncertaintyEllipse::EllipseType::k2D);
    ellipse_cov2->SetCenter(glm::vec3(6.0f, -1.0f, 0.0f));
    ellipse_cov2->SetCovarianceMatrix2D(cov2);
    ellipse_cov2->SetConfidenceLevel(UncertaintyEllipse::ConfidenceLevel::kOneSigma);
    ellipse_cov2->SetRenderMode(UncertaintyEllipse::RenderMode::kTransparent);
    ellipse_cov2->SetColor(glm::vec3(0.0f, 0.8f, 0.8f));
    ellipse_cov2->SetTransparency(0.4f);
    ellipse_cov2->SetOutlineColor(glm::vec3(0.0f, 0.5f, 0.5f));
    ellipse_cov2->SetRenderMode(UncertaintyEllipse::RenderMode::kOutlined);
    scene_manager->AddOpenGLObject("ellipse_cov_correlated", std::move(ellipse_cov2));
    
    // Center points
    auto center_cov1 = std::make_unique<Sphere>(glm::vec3(-6.0f, -1.0f, 0.0f), 0.08f);
    center_cov1->SetColor(glm::vec3(0.4f, 0.0f, 0.4f));
    scene_manager->AddOpenGLObject("center_cov1", std::move(center_cov1));
    
    auto center_cov2 = std::make_unique<Sphere>(glm::vec3(6.0f, -1.0f, 0.0f), 0.08f);
    center_cov2->SetColor(glm::vec3(0.0f, 0.5f, 0.5f));
    scene_manager->AddOpenGLObject("center_cov2", std::move(center_cov2));
}

void CreateSamplePoints(GlSceneManager* scene_manager) {
    // Generate sample points around some uncertainty ellipses to show probability distribution
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Sample points around the 2-sigma ellipse at (0, 4, 0)
    std::normal_distribution<float> dist_x(0.0f, 1.4f);  // sigma_x = 1.4
    std::normal_distribution<float> dist_y(0.0f, 0.85f); // sigma_y = 0.85
    
    std::vector<glm::vec3> sample_points;
    std::vector<glm::vec3> sample_colors;
    
    for (int i = 0; i < 200; ++i) {
        float x = dist_x(gen);
        float y = dist_y(gen);
        
        // Apply rotation (-15 degrees)
        float angle = glm::radians(-15.0f);
        float cos_a = std::cos(angle);
        float sin_a = std::sin(angle);
        
        float rotated_x = x * cos_a - y * sin_a;
        float rotated_y = x * sin_a + y * cos_a;
        
        glm::vec3 point(rotated_x + 0.0f, rotated_y + 4.0f, 0.1f);
        sample_points.push_back(point);
        
        // Color based on distance from center
        float dist_from_center = glm::length(glm::vec2(rotated_x, rotated_y));
        float normalized_dist = std::min(dist_from_center / 2.5f, 1.0f);
        
        glm::vec3 color = glm::mix(glm::vec3(1.0f, 1.0f, 0.2f), glm::vec3(0.8f, 0.2f, 0.0f), normalized_dist);
        sample_colors.push_back(color);
    }
    
    // Create point cloud
    auto point_cloud = std::make_unique<PointCloud>();
    point_cloud->SetPoints(sample_points, sample_colors);
    point_cloud->SetPointSize(3.0f);
    scene_manager->AddOpenGLObject("sample_points", std::move(point_cloud));
}

int main(int argc, char* argv[]) {
    try {
        // Configure the view
        GlView::Config config;
        config.window_title = "Uncertainty Ellipse Rendering Test";
        config.coordinate_frame_size = 2.0f;
        
        // Create the view
        GlView view(config);
        
        // Set up description and help sections
        view.SetDescription("Testing uncertainty ellipse rendering for probabilistic visualization");
        
        view.AddHelpSection("Uncertainty Types Demonstrated", {
            "✓ 2D confidence ellipses (1σ, 2σ, 3σ)",
            "✓ 3D uncertainty ellipsoids (spherical, oblate, prolate)",
            "✓ Cylindrical uncertainty volumes",
            "✓ Multi-level confidence regions",
            "✓ Covariance matrix based ellipses",
            "✓ Various render modes (filled, wireframe, gradient)",
            "✓ Sample point distributions",
            "✓ Transparency and outline effects"
        });
        
        view.AddHelpSection("Scene Contents", {
            "- Reference grid and coordinate frame",
            "- 2D ellipses: 1σ (green), 2σ (gradient), 3σ (wireframe)",
            "- 3D ellipsoids: spherical, oblate (flattened), prolate (elongated)",
            "- Cylindrical uncertainty volume",
            "- Multi-level confidence (nested 1σ, 2σ, 3σ)",
            "- Covariance-based ellipses (diagonal vs correlated)",
            "- Sample point cloud showing probability distribution"
        });
        
        view.AddHelpSection("Mathematical Features", {
            "- Chi-squared confidence level mapping",
            "- Eigenvalue decomposition for axis orientation",
            "- Mahalanobis distance calculations",
            "- Manual axis specification vs covariance matrices",
            "- Probabilistic containment testing",
            "- Multiple rendering modes and transparency"
        });
        
        // Set the scene setup callback
        view.SetSceneSetup(SetupUncertaintyScene);
        
        // Run the view
        view.Run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}