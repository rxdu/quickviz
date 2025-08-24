/*
 * @file test_measurement.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-24
 * @brief Manual test for Measurement rendering functionality
 *
 * This test creates a window displaying different measurement examples for engineering visualization.
 * Run this test to visually verify Measurement functionality for dimensional analysis.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <vector>
#include <cmath>

#include "gldraw/gl_view.hpp"
#include "gldraw/renderable/measurement.hpp"
#include "gldraw/renderable/sphere.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/coordinate_frame.hpp"

using namespace quickviz;

// Forward declarations
void CreateDistanceMeasurements(GlSceneManager* scene_manager);
void CreateAngleMeasurements(GlSceneManager* scene_manager);
void CreateRadiusDiameterMeasurements(GlSceneManager* scene_manager);
void CreateMultiSegmentMeasurements(GlSceneManager* scene_manager);
void CreateCoordinateMeasurements(GlSceneManager* scene_manager);
void CreateReferenceObjects(GlSceneManager* scene_manager);

void SetupMeasurementScene(GlSceneManager* scene_manager) {
    // Add reference objects
    CreateReferenceObjects(scene_manager);
    
    // 1. Distance measurements
    CreateDistanceMeasurements(scene_manager);
    
    // 2. Angular measurements
    CreateAngleMeasurements(scene_manager);
    
    // 3. Radius and diameter measurements
    CreateRadiusDiameterMeasurements(scene_manager);
    
    // 4. Multi-segment measurements
    CreateMultiSegmentMeasurements(scene_manager);
    
    // 5. Coordinate measurements
    CreateCoordinateMeasurements(scene_manager);
}

void CreateReferenceObjects(GlSceneManager* scene_manager) {
    // Grid for reference
    auto grid = std::make_unique<Grid>(12.0f, 1.0f, glm::vec3(0.3f, 0.3f, 0.3f));
    scene_manager->AddOpenGLObject("grid", std::move(grid));
    
    // Coordinate frame
    auto frame = std::make_unique<CoordinateFrame>(1.5f);
    scene_manager->AddOpenGLObject("frame", std::move(frame));
    
    // Reference spheres for measurement targets
    std::vector<std::pair<std::string, glm::vec3>> reference_points = {
        {"point_A", glm::vec3(-3.0f, 2.0f, 0.0f)},
        {"point_B", glm::vec3(2.0f, 3.0f, 0.0f)},
        {"point_C", glm::vec3(1.0f, -2.0f, 1.0f)},
        {"center", glm::vec3(0.0f, 0.0f, 0.0f)}
    };
    
    for (const auto& [name, pos] : reference_points) {
        auto sphere = std::make_unique<Sphere>(pos, 0.1f);
        sphere->SetColor(glm::vec3(0.8f, 0.8f, 0.2f));
        scene_manager->AddOpenGLObject("sphere_" + name, std::move(sphere));
    }
}

void CreateDistanceMeasurements(GlSceneManager* scene_manager) {
    // 1. Simple horizontal distance
    auto dist1 = std::make_unique<Measurement>(Measurement::MeasurementType::kDistance);
    dist1->SetTwoPointDistance(glm::vec3(-3.0f, 2.0f, 0.0f), glm::vec3(2.0f, 2.0f, 0.0f));
    dist1->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
    dist1->SetLineWidth(2.5f);
    dist1->SetArrowStyle(true, 0.15f);
    dist1->SetShowLabel(true);
    dist1->SetLabelPosition(Measurement::LabelPosition::kAbove);
    dist1->SetLabelColor(glm::vec3(1.0f, 0.0f, 0.0f));
    dist1->SetLabelScale(1.2f);
    scene_manager->AddOpenGLObject("distance_horizontal", std::move(dist1));
    
    // 2. Diagonal distance with extension lines
    auto dist2 = std::make_unique<Measurement>(Measurement::MeasurementType::kDistance);
    dist2->SetTwoPointDistance(glm::vec3(-3.0f, 2.0f, 0.0f), glm::vec3(2.0f, 3.0f, 0.0f));
    dist2->SetColor(glm::vec3(0.0f, 0.8f, 0.0f));
    dist2->SetLineWidth(2.0f);
    dist2->SetLineStyle(Measurement::LineStyle::kDashed);
    dist2->SetArrowStyle(true, 0.12f);
    dist2->SetExtensionLines(true, 0.3f);
    dist2->SetShowLabel(true);
    dist2->SetLabelPosition(Measurement::LabelPosition::kCenter);
    dist2->SetLabelColor(glm::vec3(0.0f, 0.8f, 0.0f));
    dist2->SetUnits("mm");
    dist2->SetPrecision(1);
    scene_manager->AddOpenGLObject("distance_diagonal", std::move(dist2));
    
    // 3. Vertical distance with custom label
    auto dist3 = std::make_unique<Measurement>(Measurement::MeasurementType::kDistance);
    dist3->SetTwoPointDistance(glm::vec3(2.0f, 3.0f, 0.0f), glm::vec3(2.0f, -2.0f, 0.0f));
    dist3->SetColor(glm::vec3(0.0f, 0.0f, 1.0f));
    dist3->SetLineWidth(3.0f);
    dist3->SetArrowStyle(true, 0.18f);
    dist3->SetShowLabel(true);
    dist3->SetLabelText("HEIGHT");
    dist3->SetLabelPosition(Measurement::LabelPosition::kCenter);
    dist3->SetLabelColor(glm::vec3(0.0f, 0.0f, 1.0f));
    dist3->SetLabelScale(1.0f);
    scene_manager->AddOpenGLObject("distance_vertical", std::move(dist3));
}

void CreateAngleMeasurements(GlSceneManager* scene_manager) {
    // 1. Acute angle measurement
    auto angle1 = std::make_unique<Measurement>(Measurement::MeasurementType::kAngle);
    glm::vec3 vertex1(0.0f, 0.0f, 0.0f);
    glm::vec3 point1a(2.0f, 0.0f, 0.0f);
    glm::vec3 point1b(1.5f, 1.5f, 0.0f);
    angle1->SetThreePointAngle(vertex1, point1a, point1b);
    angle1->SetColor(glm::vec3(1.0f, 0.5f, 0.0f));
    angle1->SetLineWidth(2.0f);
    angle1->SetArcRadius(0.8f);
    angle1->SetArcResolution(24);
    angle1->SetShowArcTicks(true, 4);
    angle1->SetShowLabel(true);
    angle1->SetLabelPosition(Measurement::LabelPosition::kCenter);
    angle1->SetLabelColor(glm::vec3(1.0f, 0.5f, 0.0f));
    angle1->SetLabelScale(1.0f);
    scene_manager->AddOpenGLObject("angle_acute", std::move(angle1));
    
    // 2. Obtuse angle measurement
    auto angle2 = std::make_unique<Measurement>(Measurement::MeasurementType::kAngle);
    glm::vec3 vertex2(-4.0f, -1.0f, 0.0f);
    glm::vec3 point2a(-2.0f, -1.0f, 0.0f);
    glm::vec3 point2b(-5.0f, 1.0f, 0.0f);
    angle2->SetThreePointAngle(vertex2, point2a, point2b);
    angle2->SetColor(glm::vec3(0.8f, 0.0f, 0.8f));
    angle2->SetLineWidth(2.5f);
    angle2->SetArcRadius(1.2f);
    angle2->SetArcResolution(32);
    angle2->SetShowArcTicks(true, 6);
    angle2->SetShowLabel(true);
    angle2->SetLabelPosition(Measurement::LabelPosition::kCenter);
    angle2->SetLabelColor(glm::vec3(0.8f, 0.0f, 0.8f));
    angle2->SetPrecision(1);
    scene_manager->AddOpenGLObject("angle_obtuse", std::move(angle2));
}

void CreateRadiusDiameterMeasurements(GlSceneManager* scene_manager) {
    // Reference circle visualization using spheres
    glm::vec3 circle_center(4.0f, 0.0f, 0.0f);
    float circle_radius = 1.8f;
    
    // Create circle outline with small spheres
    for (int i = 0; i < 16; ++i) {
        float angle = i * 2.0f * M_PI / 16.0f;
        glm::vec3 pos = circle_center + glm::vec3(
            circle_radius * std::cos(angle),
            circle_radius * std::sin(angle),
            0.0f
        );
        auto sphere = std::make_unique<Sphere>(pos, 0.05f);
        sphere->SetColor(glm::vec3(0.5f, 0.5f, 0.5f));
        scene_manager->AddOpenGLObject("circle_point_" + std::to_string(i), std::move(sphere));
    }
    
    // Center point
    auto center_sphere = std::make_unique<Sphere>(circle_center, 0.08f);
    center_sphere->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
    scene_manager->AddOpenGLObject("circle_center", std::move(center_sphere));
    
    // 1. Radius measurement
    auto radius = std::make_unique<Measurement>(Measurement::MeasurementType::kRadius);
    glm::vec3 radius_point = circle_center + glm::vec3(circle_radius, 0.0f, 0.0f);
    radius->SetRadius(circle_center, radius_point);
    radius->SetColor(glm::vec3(0.0f, 1.0f, 1.0f));
    radius->SetLineWidth(3.0f);
    radius->SetArrowStyle(true, 0.15f);
    radius->SetShowLabel(true);
    radius->SetLabelPosition(Measurement::LabelPosition::kCenter);
    radius->SetLabelColor(glm::vec3(0.0f, 1.0f, 1.0f));
    radius->SetLabelScale(1.1f);
    scene_manager->AddOpenGLObject("radius_measurement", std::move(radius));
    
    // 2. Diameter measurement
    auto diameter = std::make_unique<Measurement>(Measurement::MeasurementType::kDiameter);
    glm::vec3 diam_point1 = circle_center + glm::vec3(0.0f, circle_radius, 0.0f);
    glm::vec3 diam_point2 = circle_center + glm::vec3(0.0f, -circle_radius, 0.0f);
    diameter->SetDiameter(circle_center, diam_point1, diam_point2);
    diameter->SetColor(glm::vec3(1.0f, 0.0f, 1.0f));
    diameter->SetLineWidth(2.5f);
    diameter->SetLineStyle(Measurement::LineStyle::kDashDot);
    diameter->SetArrowStyle(true, 0.12f);
    diameter->SetShowLabel(true);
    diameter->SetLabelPosition(Measurement::LabelPosition::kStart);
    diameter->SetLabelColor(glm::vec3(1.0f, 0.0f, 1.0f));
    diameter->SetLabelScale(1.0f);
    scene_manager->AddOpenGLObject("diameter_measurement", std::move(diameter));
}

void CreateMultiSegmentMeasurements(GlSceneManager* scene_manager) {
    // Path with multiple segments
    std::vector<glm::vec3> path_points = {
        glm::vec3(-5.0f, -3.0f, 0.0f),
        glm::vec3(-3.0f, -4.0f, 0.5f),
        glm::vec3(-1.0f, -3.5f, 1.0f),
        glm::vec3(1.0f, -4.0f, 0.8f),
        glm::vec3(3.0f, -3.0f, 0.0f)
    };
    
    // Add small spheres at path points
    for (size_t i = 0; i < path_points.size(); ++i) {
        auto sphere = std::make_unique<Sphere>(path_points[i], 0.06f);
        sphere->SetColor(glm::vec3(0.9f, 0.7f, 0.1f));
        scene_manager->AddOpenGLObject("path_point_" + std::to_string(i), std::move(sphere));
    }
    
    // Multi-segment measurement
    auto multi_seg = std::make_unique<Measurement>(Measurement::MeasurementType::kMultiSegment);
    multi_seg->SetPoints(path_points);
    multi_seg->SetColor(glm::vec3(0.9f, 0.7f, 0.1f));
    multi_seg->SetLineWidth(2.8f);
    multi_seg->SetArrowStyle(true, 0.1f);
    multi_seg->SetShowLabel(true);
    multi_seg->SetLabelText("TOTAL PATH");
    multi_seg->SetLabelPosition(Measurement::LabelPosition::kCenter);
    multi_seg->SetLabelColor(glm::vec3(0.9f, 0.7f, 0.1f));
    multi_seg->SetLabelScale(1.2f);
    multi_seg->SetUnits("cm");
    multi_seg->SetPrecision(1);
    scene_manager->AddOpenGLObject("multi_segment", std::move(multi_seg));
}

void CreateCoordinateMeasurements(GlSceneManager* scene_manager) {
    // X-coordinate measurement
    auto coord_x = std::make_unique<Measurement>(Measurement::MeasurementType::kCoordinate);
    glm::vec3 target_point(-2.0f, -1.0f, 2.0f);
    coord_x->SetCoordinate(glm::vec3(0.0f, -1.0f, 2.0f), glm::vec3(-1.0f, 0.0f, 0.0f), 2.0f);
    coord_x->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
    coord_x->SetLineWidth(2.0f);
    coord_x->SetLineStyle(Measurement::LineStyle::kDotted);
    coord_x->SetArrowStyle(false);
    coord_x->SetShowLabel(true);
    coord_x->SetLabelText("X = -2.0");
    coord_x->SetLabelPosition(Measurement::LabelPosition::kEnd);
    coord_x->SetLabelColor(glm::vec3(1.0f, 0.0f, 0.0f));
    coord_x->SetLabelScale(0.8f);
    scene_manager->AddOpenGLObject("coordinate_x", std::move(coord_x));
    
    // Target point sphere
    auto target_sphere = std::make_unique<Sphere>(target_point, 0.12f);
    target_sphere->SetColor(glm::vec3(1.0f, 1.0f, 0.0f));
    scene_manager->AddOpenGLObject("target_point", std::move(target_sphere));
    
    // Z-coordinate measurement
    auto coord_z = std::make_unique<Measurement>(Measurement::MeasurementType::kCoordinate);
    coord_z->SetCoordinate(glm::vec3(-2.0f, -1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), 2.0f);
    coord_z->SetColor(glm::vec3(0.0f, 0.0f, 1.0f));
    coord_z->SetLineWidth(2.0f);
    coord_z->SetLineStyle(Measurement::LineStyle::kDotted);
    coord_z->SetArrowStyle(false);
    coord_z->SetShowLabel(true);
    coord_z->SetLabelText("Z = 2.0");
    coord_z->SetLabelPosition(Measurement::LabelPosition::kEnd);
    coord_z->SetLabelColor(glm::vec3(0.0f, 0.0f, 1.0f));
    coord_z->SetLabelScale(0.8f);
    scene_manager->AddOpenGLObject("coordinate_z", std::move(coord_z));
}

int main(int argc, char* argv[]) {
    try {
        // Configure the view
        GlView::Config config;
        config.window_title = "Measurement Rendering Test";
        config.coordinate_frame_size = 2.0f;
        
        // Create the view
        GlView view(config);
        
        // Set up description and help sections
        view.SetDescription("Testing measurement rendering for engineering and robotics visualization");
        
        view.AddHelpSection("Measurement Types Demonstrated", {
            "✓ Distance measurements with arrows and labels",
            "✓ Angular measurements with arcs and tick marks",
            "✓ Radius and diameter measurements",
            "✓ Multi-segment path measurements",
            "✓ Coordinate value displays",
            "✓ Various line styles (solid, dashed, dotted)",
            "✓ Extension lines and dimensional callouts",
            "✓ Custom labels and precision control"
        });
        
        view.AddHelpSection("Scene Contents", {
            "- Reference grid and coordinate frame",
            "- Distance measurements (horizontal, diagonal, vertical)",
            "- Angular measurements (acute, obtuse angles)",
            "- Radius and diameter of circle",
            "- Multi-segment path with total length",
            "- Coordinate measurements (X, Z values)",
            "- Reference spheres marking measurement points"
        });
        
        view.AddHelpSection("Features Tested", {
            "- Multiple measurement types and modes",
            "- Automatic and custom label generation", 
            "- Line styles and arrow configurations",
            "- Label positioning and formatting",
            "- Units and precision control",
            "- Color coding and visual hierarchy"
        });
        
        // Set the scene setup callback
        view.SetSceneSetup(SetupMeasurementScene);
        
        // Run the view
        view.Run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}