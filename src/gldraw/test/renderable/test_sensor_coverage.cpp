/*
 * @file test_sensor_coverage.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-24
 * @brief Test for SensorCoverage rendering functionality
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <vector>
#include <cmath>

#include "gldraw/gl_view.hpp"
#include "gldraw/renderable/sensor_coverage.hpp"
#include "gldraw/renderable/sphere.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/coordinate_frame.hpp"
#include "gldraw/renderable/arrow.hpp"
#include "gldraw/renderable/text3d.hpp"

using namespace quickviz;

// Forward declarations
void CreateLidarCoverage(GlSceneManager* scene_manager);
void CreateCameraCoverage(GlSceneManager* scene_manager);
void CreateRadarCoverage(GlSceneManager* scene_manager);
void CreateSonarCoverage(GlSceneManager* scene_manager);
void CreateProximitySensors(GlSceneManager* scene_manager);
void CreateMultiSensorSetup(GlSceneManager* scene_manager);
void CreateReferenceObjects(GlSceneManager* scene_manager);

void SetupSensorCoverageScene(GlSceneManager* scene_manager) {
    // Add reference objects
    CreateReferenceObjects(scene_manager);
    
    // 1. LIDAR sensor coverage
    CreateLidarCoverage(scene_manager);

    // 2. Camera sensor coverage
    CreateCameraCoverage(scene_manager);

    // 3. Radar sensor coverage
    CreateRadarCoverage(scene_manager);

    // 4. Sonar sensor coverage
    CreateSonarCoverage(scene_manager);

    // 5. Proximity sensors
    CreateProximitySensors(scene_manager);

    // // 6. Multi-sensor robot setup
    // CreateMultiSensorSetup(scene_manager);
}

void CreateReferenceObjects(GlSceneManager* scene_manager) {
    // Large grid for sensor range visualization
    auto grid = std::make_unique<Grid>(40.0f, 2.0f, glm::vec3(0.25f, 0.25f, 0.25f));
    scene_manager->AddOpenGLObject("grid", std::move(grid));
    
    // Coordinate frame at origin
    auto frame = std::make_unique<CoordinateFrame>(2.0f);
    scene_manager->AddOpenGLObject("frame", std::move(frame));
    
    // Target objects for sensor detection
    std::vector<std::pair<glm::vec3, glm::vec3>> targets = {
        {glm::vec3(8.0f, 2.0f, 0.5f), glm::vec3(1.0f, 0.3f, 0.3f)},   // Red target
        {glm::vec3(5.0f, -6.0f, 1.0f), glm::vec3(0.3f, 1.0f, 0.3f)},  // Green target
        {glm::vec3(-4.0f, 7.0f, 0.8f), glm::vec3(0.3f, 0.3f, 1.0f)},  // Blue target
        {glm::vec3(-8.0f, -3.0f, 0.3f), glm::vec3(1.0f, 1.0f, 0.3f)}  // Yellow target
    };
    
    for (size_t i = 0; i < targets.size(); ++i) {
        auto target = std::make_unique<Sphere>(targets[i].first, 0.3f);
        target->SetColor(targets[i].second);
        scene_manager->AddOpenGLObject("target_" + std::to_string(i), std::move(target));
    }
}

void CreateLidarCoverage(GlSceneManager* scene_manager) {
    // 2D LIDAR - Left side
    auto lidar_2d = std::make_unique<SensorCoverage>(SensorCoverage::SensorType::kLidar);
    lidar_2d->SetSensorPosition(glm::vec3(-8.0f, 15.0f, 0.5f));
    // Set orientation to keep rings on horizontal plane (Z=0)
    lidar_2d->SetSensorOrientation(glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    lidar_2d->SetRange(0.1f, 8.0f);
    lidar_2d->SetRangeRingCount(4);
    lidar_2d->SetCoverageType(SensorCoverage::CoverageType::k2DRings);
    lidar_2d->SetVisualizationMode(SensorCoverage::VisualizationMode::kRangeRings);
    lidar_2d->SetColor(glm::vec3(0.0f, 0.8f, 0.2f));
    lidar_2d->SetTransparency(0.4f);
    scene_manager->AddOpenGLObject("lidar_2d", std::move(lidar_2d));
    
    // LIDAR sensor indicator
    auto lidar_sensor = std::make_unique<Sphere>(glm::vec3(-8.0f, 15.0f, 0.5f), 0.15f);
    lidar_sensor->SetColor(glm::vec3(0.0f, 0.6f, 0.0f));
    scene_manager->AddOpenGLObject("lidar_sensor", std::move(lidar_sensor));
    
    // Scanning LIDAR with limited FOV - Right side
    auto lidar_scanning = std::make_unique<SensorCoverage>(SensorCoverage::SensorType::kLidar);
    lidar_scanning->SetSensorPosition(glm::vec3(8.0f, 15.0f, 0.5f));  // Same Z as 2D LIDAR
    // Set orientation to keep rings on horizontal plane (Z=0)
    lidar_scanning->SetSensorOrientation(glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    lidar_scanning->SetRange(0.2f, 6.0f);
    lidar_scanning->SetAngularCoverage(glm::radians(120.0f));  // 120-degree FOV
    // Center the angular coverage: -60° to +60° relative to forward direction
    lidar_scanning->SetAngularLimits(glm::radians(-60.0f), glm::radians(60.0f));
    lidar_scanning->SetRangeRingCount(3);
    lidar_scanning->SetCoverageType(SensorCoverage::CoverageType::k2DRings);
    lidar_scanning->SetVisualizationMode(SensorCoverage::VisualizationMode::kRangeRings);
    lidar_scanning->SetColor(glm::vec3(0.2f, 1.0f, 0.4f));
    lidar_scanning->SetTransparency(0.5f);
    scene_manager->AddOpenGLObject("lidar_scanning", std::move(lidar_scanning));
    
    // Scanning LIDAR sensor indicator
    auto scanning_sensor = std::make_unique<Sphere>(glm::vec3(8.0f, 15.0f, 0.5f), 0.12f);
    scanning_sensor->SetColor(glm::vec3(0.1f, 0.8f, 0.2f));
    scene_manager->AddOpenGLObject("scanning_sensor", std::move(scanning_sensor));
    
    // Direction arrow
    auto lidar_arrow = std::make_unique<Arrow>();
    lidar_arrow->SetStartPoint(glm::vec3(8.0f, 15.0f, 0.5f));
    lidar_arrow->SetEndPoint(glm::vec3(8.0f - 1.0f, 15.0f - 1.0f, 0.5f));
    lidar_arrow->SetColor(glm::vec3(0.1f, 0.8f, 0.2f));
    scene_manager->AddOpenGLObject("lidar_direction", std::move(lidar_arrow));
    
    // LIDAR labels
    auto lidar_2d_label = std::make_unique<Text3D>();
    lidar_2d_label->SetText("2D LIDAR");
    lidar_2d_label->SetPosition(glm::vec3(-8.0f, 15.0f, 3.0f));
    lidar_2d_label->SetColor(glm::vec3(0.0f, 1.0f, 0.3f));
    lidar_2d_label->SetScale(0.8f);
    scene_manager->AddOpenGLObject("lidar_2d_label", std::move(lidar_2d_label));
    
    auto lidar_scan_label = std::make_unique<Text3D>();
    lidar_scan_label->SetText("SCANNING LIDAR");
    lidar_scan_label->SetPosition(glm::vec3(8.0f, 15.0f, 3.0f));
    lidar_scan_label->SetColor(glm::vec3(0.2f, 1.0f, 0.4f));
    lidar_scan_label->SetScale(0.8f);
    scene_manager->AddOpenGLObject("lidar_scan_label", std::move(lidar_scan_label));
}

void CreateCameraCoverage(GlSceneManager* scene_manager) {
    // RGB camera - positioned at Y=0 in scene reference frame
    auto camera = std::make_unique<SensorCoverage>(SensorCoverage::SensorType::kCamera);
    camera->SetSensorPosition(glm::vec3(-12.0f, 0.0f, 2.0f));  // Y=0 in scene reference frame
    // Default orientation: cone points along +Z (forward in local space)
    camera->SetSensorOrientation(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    camera->SetRange(0.1f, 6.0f);
    camera->SetAngularCoverage(glm::radians(60.0f), glm::radians(45.0f));  // H x V FOV
    camera->SetVisualizationMode(SensorCoverage::VisualizationMode::kSolid);
    camera->SetColor(glm::vec3(0.3f, 0.5f, 1.0f));
    camera->SetTransparency(0.4f);
    scene_manager->AddOpenGLObject("camera", std::move(camera));
    
    // Camera sensor body
    auto camera_body = std::make_unique<Sphere>(glm::vec3(-12.0f, 0.0f, 2.0f), 0.15f);
    camera_body->SetColor(glm::vec3(0.1f, 0.3f, 0.8f));
    scene_manager->AddOpenGLObject("camera_body", std::move(camera_body));
    
    // Camera direction indicator - should point in +Y direction
    auto camera_arrow = std::make_unique<Arrow>();
    camera_arrow->SetStartPoint(glm::vec3(-12.0f, 0.0f, 2.0f));
    camera_arrow->SetEndPoint(glm::vec3(-12.0f, 2.0f, 2.0f));  // Point forward (+Y)
    camera_arrow->SetColor(glm::vec3(0.1f, 0.3f, 0.8f));
    scene_manager->AddOpenGLObject("camera_direction", std::move(camera_arrow));
    
    // Security camera - positioned at Y=0 in scene reference frame
    auto security_cam = std::make_unique<SensorCoverage>(SensorCoverage::SensorType::kCamera);
    security_cam->SetSensorPosition(glm::vec3(12.0f, 0.0f, 2.5f));  // Y=0 in scene reference frame
    // Default orientation: cone points along +Z (forward in local space)
    security_cam->SetSensorOrientation(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    security_cam->SetRange(0.5f, 6.0f);
    security_cam->SetAngularCoverage(glm::radians(80.0f), glm::radians(50.0f));
    security_cam->SetVisualizationMode(SensorCoverage::VisualizationMode::kSolid);
    security_cam->SetColor(glm::vec3(0.6f, 0.4f, 1.0f));
    security_cam->SetTransparency(0.6f);
    security_cam->SetCoverageType(SensorCoverage::CoverageType::k3DCone);
    scene_manager->AddOpenGLObject("security_camera", std::move(security_cam));
    
    // Security camera body
    auto sec_cam_body = std::make_unique<Sphere>(glm::vec3(12.0f, 0.0f, 2.5f), 0.12f);
    sec_cam_body->SetColor(glm::vec3(0.4f, 0.2f, 0.8f));
    scene_manager->AddOpenGLObject("security_camera_body", std::move(sec_cam_body));
    
    // Security camera direction indicator - should point in +Y direction
    auto security_arrow = std::make_unique<Arrow>();
    security_arrow->SetStartPoint(glm::vec3(12.0f, 0.0f, 2.5f));
    security_arrow->SetEndPoint(glm::vec3(12.0f, 2.0f, 2.5f));  // Point forward (+Y)
    security_arrow->SetColor(glm::vec3(0.4f, 0.2f, 0.8f));
    scene_manager->AddOpenGLObject("security_direction", std::move(security_arrow));
    
    // Camera labels
    auto camera_label = std::make_unique<Text3D>();
    camera_label->SetText("RGB CAMERA");
    camera_label->SetPosition(glm::vec3(-12.0f, 0.0f, 4.0f));
    camera_label->SetColor(glm::vec3(0.3f, 0.5f, 1.0f));
    camera_label->SetScale(0.8f);
    scene_manager->AddOpenGLObject("camera_label", std::move(camera_label));
    
    auto security_label = std::make_unique<Text3D>();
    security_label->SetText("SECURITY CAM");
    security_label->SetPosition(glm::vec3(12.0f, 0.0f, 4.5f));
    security_label->SetColor(glm::vec3(0.6f, 0.4f, 1.0f));
    security_label->SetScale(0.8f);
    scene_manager->AddOpenGLObject("security_label", std::move(security_label));
}

void CreateRadarCoverage(GlSceneManager* scene_manager) {
    // Automotive radar sensor - Left side
    auto radar = std::make_unique<SensorCoverage>(SensorCoverage::SensorType::kRadar);
    radar->SetSensorPosition(glm::vec3(-15.0f, 0.0f, 1.0f));
    radar->SetSensorOrientation(glm::vec3(1.0f, 0.2f, 0.0f));
    radar->SetRange(2.0f, 25.0f);
    radar->SetAngularCoverage(glm::radians(90.0f), glm::radians(15.0f));
    radar->SetVisualizationMode(SensorCoverage::VisualizationMode::kHeatMap);
    radar->SetRangeColors(glm::vec3(1.0f, 0.8f, 0.2f), glm::vec3(1.0f, 0.2f, 0.0f));
    radar->SetTransparency(0.3f);
    radar->SetRangeRingCount(3);
    scene_manager->AddOpenGLObject("radar", std::move(radar));
    
    // Radar sensor housing
    auto radar_housing = std::make_unique<Sphere>(glm::vec3(-15.0f, 0.0f, 1.0f), 0.18f);
    radar_housing->SetColor(glm::vec3(0.8f, 0.4f, 0.0f));
    scene_manager->AddOpenGLObject("radar_housing", std::move(radar_housing));
    
    // Radar direction
    auto radar_arrow = std::make_unique<Arrow>();
    radar_arrow->SetStartPoint(glm::vec3(-15.0f, 0.0f, 1.0f));
    glm::vec3 radar_end = glm::vec3(-15.0f, 0.0f, 1.0f) + 1.2f * glm::normalize(glm::vec3(1.0f, 0.2f, 0.0f));
    radar_arrow->SetEndPoint(radar_end);
    radar_arrow->SetColor(glm::vec3(0.8f, 0.4f, 0.0f));
    scene_manager->AddOpenGLObject("radar_direction", std::move(radar_arrow));
    
    // Long-range radar - Right side
    auto long_range_radar = std::make_unique<SensorCoverage>(SensorCoverage::SensorType::kRadar);
    long_range_radar->SetSensorPosition(glm::vec3(15.0f, 0.0f, 0.8f));
    long_range_radar->SetSensorOrientation(glm::vec3(-1.0f, 0.1f, 0.05f));
    long_range_radar->SetRange(5.0f, 40.0f);
    long_range_radar->SetAngularCoverage(glm::radians(45.0f), glm::radians(10.0f));
    long_range_radar->SetVisualizationMode(SensorCoverage::VisualizationMode::kTransparent);
    long_range_radar->SetColor(glm::vec3(1.0f, 0.5f, 0.1f));
    long_range_radar->SetTransparency(0.2f);
    scene_manager->AddOpenGLObject("long_range_radar", std::move(long_range_radar));
    
    // Long-range radar body
    auto lr_radar_body = std::make_unique<Sphere>(glm::vec3(15.0f, 0.0f, 0.8f), 0.12f);
    lr_radar_body->SetColor(glm::vec3(0.8f, 0.3f, 0.0f));
    scene_manager->AddOpenGLObject("lr_radar_body", std::move(lr_radar_body));
    
    // Radar labels
    auto radar_label = std::make_unique<Text3D>();
    radar_label->SetText("AUTO RADAR");
    radar_label->SetPosition(glm::vec3(-15.0f, 0.0f, 4.0f));
    radar_label->SetColor(glm::vec3(1.0f, 0.6f, 0.0f));
    radar_label->SetScale(0.8f);
    scene_manager->AddOpenGLObject("radar_label", std::move(radar_label));
    
    auto lr_radar_label = std::make_unique<Text3D>();
    lr_radar_label->SetText("LONG RANGE");
    lr_radar_label->SetPosition(glm::vec3(15.0f, 0.0f, 4.0f));
    lr_radar_label->SetColor(glm::vec3(1.0f, 0.5f, 0.1f));
    lr_radar_label->SetScale(0.8f);
    scene_manager->AddOpenGLObject("lr_radar_label", std::move(lr_radar_label));
}

void CreateSonarCoverage(GlSceneManager* scene_manager) {
    // Underwater sonar array - Top center
    std::vector<glm::vec3> sonar_positions = {
        glm::vec3(-3.0f, 15.0f, 0.5f),
        glm::vec3(-1.0f, 15.0f, 0.5f),
        glm::vec3(1.0f, 15.0f, 0.5f),
        glm::vec3(3.0f, 15.0f, 0.5f)
    };
    
    std::vector<glm::vec3> sonar_directions = {
        glm::vec3(0.3f, -1.0f, 0.0f),
        glm::vec3(0.1f, -1.0f, 0.0f),
        glm::vec3(-0.1f, -1.0f, 0.0f),
        glm::vec3(-0.3f, -1.0f, 0.0f)
    };
    
    for (size_t i = 0; i < sonar_positions.size(); ++i) {
        auto sonar = std::make_unique<SensorCoverage>(SensorCoverage::SensorType::kSonar);
        sonar->SetSensorPosition(sonar_positions[i]);
        sonar->SetSensorOrientation(sonar_directions[i]);
        sonar->SetRange(0.1f, 6.0f);
        sonar->SetAngularCoverage(glm::radians(45.0f), glm::radians(30.0f));
        sonar->SetVisualizationMode(SensorCoverage::VisualizationMode::kTransparent);
        sonar->SetColor(glm::vec3(0.8f, 0.2f, 0.8f));
        sonar->SetTransparency(0.4f);
        scene_manager->AddOpenGLObject("sonar_" + std::to_string(i), std::move(sonar));
        
        // Sonar transducer
        auto transducer = std::make_unique<Sphere>(sonar_positions[i], 0.08f);
        transducer->SetColor(glm::vec3(0.6f, 0.0f, 0.6f));
        scene_manager->AddOpenGLObject("transducer_" + std::to_string(i), std::move(transducer));
    }
    
    // Sonar label
    auto sonar_label = std::make_unique<Text3D>();
    sonar_label->SetText("SONAR ARRAY");
    sonar_label->SetPosition(glm::vec3(0.0f, 15.0f, 3.0f));
    sonar_label->SetColor(glm::vec3(0.8f, 0.2f, 0.8f));
    sonar_label->SetScale(0.8f);
    scene_manager->AddOpenGLObject("sonar_label", std::move(sonar_label));
}

void CreateProximitySensors(GlSceneManager* scene_manager) {
    // Proximity sensors around a robot platform - Bottom center
    std::vector<glm::vec3> prox_positions = {
        glm::vec3(-2.0f, -15.0f, 0.2f),   // Front-left  
        glm::vec3(2.0f, -15.0f, 0.2f),    // Front-right
        glm::vec3(-2.0f, -17.0f, 0.2f),   // Back-left
        glm::vec3(2.0f, -17.0f, 0.2f),    // Back-right
    };
    
    for (size_t i = 0; i < prox_positions.size(); ++i) {
        auto proximity = std::make_unique<SensorCoverage>(SensorCoverage::SensorType::kProximity);
        proximity->SetSensorPosition(prox_positions[i]);
        proximity->SetRange(0.02f, 1.2f);
        proximity->SetVisualizationMode(SensorCoverage::VisualizationMode::kTransparent);
        proximity->SetColor(glm::vec3(1.0f, 1.0f, 0.3f));
        proximity->SetTransparency(0.5f);
        scene_manager->AddOpenGLObject("proximity_" + std::to_string(i), std::move(proximity));
        
        // Proximity sensor body
        auto prox_body = std::make_unique<Sphere>(prox_positions[i], 0.05f);
        prox_body->SetColor(glm::vec3(0.8f, 0.8f, 0.0f));
        scene_manager->AddOpenGLObject("prox_body_" + std::to_string(i), std::move(prox_body));
    }
    
    // Robot platform representation
    auto platform = std::make_unique<Sphere>(glm::vec3(0.0f, -16.0f, 0.1f), 0.6f);
    platform->SetColor(glm::vec3(0.4f, 0.4f, 0.4f));
    scene_manager->AddOpenGLObject("robot_platform", std::move(platform));
    
    // Proximity sensor label
    auto proximity_label = std::make_unique<Text3D>();
    proximity_label->SetText("PROXIMITY");
    proximity_label->SetPosition(glm::vec3(0.0f, -15.0f, 3.0f));
    proximity_label->SetColor(glm::vec3(1.0f, 1.0f, 0.3f));
    proximity_label->SetScale(0.8f);
    scene_manager->AddOpenGLObject("proximity_label", std::move(proximity_label));
}

void CreateMultiSensorSetup(GlSceneManager* scene_manager) {
    // Autonomous vehicle sensor suite - Center
    glm::vec3 vehicle_pos(0.0f, 0.0f, 0.5f);
    
    // Vehicle body
    auto vehicle = std::make_unique<Sphere>(vehicle_pos, 0.8f);
    vehicle->SetColor(glm::vec3(0.3f, 0.3f, 0.3f));
    scene_manager->AddOpenGLObject("vehicle_body", std::move(vehicle));
    
    // Front camera
    auto front_cam = std::make_unique<SensorCoverage>(SensorCoverage::SensorType::kCamera);
    front_cam->SetSensorPosition(vehicle_pos + glm::vec3(0.8f, 0.0f, 0.2f));
    front_cam->SetSensorOrientation(glm::vec3(1.0f, 0.0f, 0.0f));
    front_cam->SetRange(0.5f, 25.0f);
    front_cam->SetAngularCoverage(glm::radians(80.0f), glm::radians(45.0f));
    front_cam->SetVisualizationMode(SensorCoverage::VisualizationMode::kTransparent);
    front_cam->SetColor(glm::vec3(0.4f, 0.6f, 1.0f));
    front_cam->SetTransparency(0.2f);
    scene_manager->AddOpenGLObject("vehicle_front_cam", std::move(front_cam));
    
    // Side radar
    auto side_radar = std::make_unique<SensorCoverage>(SensorCoverage::SensorType::kRadar);
    side_radar->SetSensorPosition(vehicle_pos + glm::vec3(0.0f, 0.8f, 0.0f));
    side_radar->SetSensorOrientation(glm::vec3(0.0f, 1.0f, 0.0f));
    side_radar->SetRange(1.0f, 15.0f);
    side_radar->SetAngularCoverage(glm::radians(90.0f), glm::radians(25.0f));
    side_radar->SetVisualizationMode(SensorCoverage::VisualizationMode::kRangeRings);
    side_radar->SetColor(glm::vec3(1.0f, 0.6f, 0.2f));
    side_radar->SetTransparency(0.4f);
    side_radar->SetRangeRingCount(3);
    scene_manager->AddOpenGLObject("vehicle_side_radar", std::move(side_radar));
    
    // Top LIDAR
    auto top_lidar = std::make_unique<SensorCoverage>(SensorCoverage::SensorType::kLidar);
    top_lidar->SetSensorPosition(vehicle_pos + glm::vec3(0.0f, 0.0f, 1.0f));
    top_lidar->SetRange(0.1f, 20.0f);
    top_lidar->SetRangeRingCount(4);
    top_lidar->SetVisualizationMode(SensorCoverage::VisualizationMode::kRangeRings);
    top_lidar->SetColor(glm::vec3(0.2f, 1.0f, 0.4f));
    top_lidar->SetTransparency(0.3f);
    scene_manager->AddOpenGLObject("vehicle_top_lidar", std::move(top_lidar));
    
    // LIDAR housing
    auto lidar_housing = std::make_unique<Sphere>(vehicle_pos + glm::vec3(0.0f, 0.0f, 1.0f), 0.2f);
    lidar_housing->SetColor(glm::vec3(0.1f, 0.8f, 0.2f));
    scene_manager->AddOpenGLObject("vehicle_lidar_housing", std::move(lidar_housing));
    
    // Multi-sensor label
    auto vehicle_label = std::make_unique<Text3D>();
    vehicle_label->SetText("MULTI-SENSOR");
    vehicle_label->SetPosition(glm::vec3(0.0f, 0.0f, 4.0f));
    vehicle_label->SetColor(glm::vec3(0.8f, 0.8f, 0.8f));
    vehicle_label->SetScale(0.8f);
    scene_manager->AddOpenGLObject("vehicle_label", std::move(vehicle_label));
}

int main(int argc, char* argv[]) {
    try {
        // Configure the view
        GlView::Config config;
        config.window_title = "Sensor Coverage Rendering Test";
        config.coordinate_frame_size = 2.0f;
        
        // Create the view
        GlView view(config);
        
        // Set up description and help sections
        view.SetDescription("Testing sensor coverage rendering for robotics sensor visualization");
        
        view.AddHelpSection("Sensor Types Demonstrated", {
            "✓ LIDAR: 360° and scanning with range rings",
            "✓ Camera: Perspective FOV cones (RGB, security)",
            "✓ Radar: Detection cones with heat maps",
            "✓ Sonar: Multi-beam acoustic arrays",
            "✓ Proximity: Spherical detection zones",
            "✓ Multi-sensor: Autonomous vehicle setup",
            "✓ Various visualization modes and transparency",
            "✓ Range indicators and direction arrows"
        });
        
        view.AddHelpSection("Scene Contents", {
            "- Reference grid and coordinate frame",
            "- LIDAR: Full 360° at origin, 180° scanning sensor",
            "- Cameras: RGB with perspective FOV, security camera",
            "- Radar: Automotive short/long range sensors", 
            "- Sonar: 4-element underwater array",
            "- Proximity: Robot platform with 4 sensors",
            "- Multi-sensor: Autonomous vehicle with LIDAR/camera/radar",
            "- Target objects for detection scenarios"
        });
        
        view.AddHelpSection("Visualization Features", {
            "- Range rings and distance indicators",
            "- 3D coverage volumes and cones",
            "- Heat map and transparency effects",
            "- Sensor orientation arrows",
            "- Multiple render modes (wireframe, solid, transparent)",
            "- Color coding by sensor type and range",
            "- Realistic sensor parameters and coverage patterns"
        });
        
        // Set the scene setup callback
        view.SetSceneSetup(SetupSensorCoverageScene);
        
        // Run the view
        view.Run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}