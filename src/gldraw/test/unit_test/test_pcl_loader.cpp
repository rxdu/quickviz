/*
 * @file test_pcl_loader.cpp
 * @date Dec 2024
 * @brief Unit tests for PCL point cloud loader with automatic field detection
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <memory>

#include "gldraw/pcl_bridge/pcl_loader.hpp"
#include "gldraw/renderable/point_cloud.hpp"

#ifdef QUICKVIZ_WITH_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#endif

namespace quickviz {
namespace pcl_bridge {
namespace test {

class PCLLoaderTest : public ::testing::Test {
 protected:
  void SetUp() override {
#ifdef QUICKVIZ_WITH_PCL
    test_dir_ = std::filesystem::temp_directory_path() / "quickviz_pcl_loader_test";
    std::filesystem::create_directories(test_dir_);
    
    CreateTestPCDFiles();
    CreateTestPLYFiles();
#endif
  }

  void TearDown() override {
#ifdef QUICKVIZ_WITH_PCL
    if (std::filesystem::exists(test_dir_)) {
      std::filesystem::remove_all(test_dir_);
    }
#endif
  }

#ifdef QUICKVIZ_WITH_PCL
  void CreateTestPCDFiles() {
    // Create test PCD file with XYZ points
    CreateTestPCDXYZ();
    
    // Create test PCD file with XYZI points
    CreateTestPCDXYZI();
    
    // Create test PCD file with XYZRGB points
    CreateTestPCDXYZRGB();
    
    // Create invalid PCD file
    CreateInvalidPCD();
  }

  void CreateTestPCDXYZ() {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 100;
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.points.resize(cloud.width * cloud.height);

    for (size_t i = 0; i < cloud.points.size(); ++i) {
      cloud.points[i].x = static_cast<float>(i) * 0.1f;
      cloud.points[i].y = static_cast<float>(i) * 0.2f;
      cloud.points[i].z = static_cast<float>(i) * 0.3f;
    }

    std::string filename = test_dir_ / "test_xyz.pcd";
    pcl::io::savePCDFileASCII(filename, cloud);
    test_pcd_xyz_ = filename;
  }

  void CreateTestPCDXYZI() {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.width = 50;
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.points.resize(cloud.width * cloud.height);

    for (size_t i = 0; i < cloud.points.size(); ++i) {
      cloud.points[i].x = static_cast<float>(i) * 0.1f;
      cloud.points[i].y = static_cast<float>(i) * 0.2f;
      cloud.points[i].z = static_cast<float>(i) * 0.3f;
      cloud.points[i].intensity = static_cast<float>(i) / 255.0f;
    }

    std::string filename = test_dir_ / "test_xyzi.pcd";
    pcl::io::savePCDFileASCII(filename, cloud);
    test_pcd_xyzi_ = filename;
  }

  void CreateTestPCDXYZRGB() {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.width = 75;
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.points.resize(cloud.width * cloud.height);

    for (size_t i = 0; i < cloud.points.size(); ++i) {
      cloud.points[i].x = static_cast<float>(i) * 0.1f;
      cloud.points[i].y = static_cast<float>(i) * 0.2f;
      cloud.points[i].z = static_cast<float>(i) * 0.3f;
      cloud.points[i].r = static_cast<uint8_t>(i % 256);
      cloud.points[i].g = static_cast<uint8_t>((i * 2) % 256);
      cloud.points[i].b = static_cast<uint8_t>((i * 3) % 256);
    }

    std::string filename = test_dir_ / "test_xyzrgb.pcd";
    pcl::io::savePCDFileASCII(filename, cloud);
    test_pcd_xyzrgb_ = filename;
  }

  void CreateInvalidPCD() {
    std::string filename = test_dir_ / "invalid.pcd";
    std::ofstream file(filename);
    file << "This is not a valid PCD file\n";
    file << "Invalid content\n";
    file.close();
    test_invalid_pcd_ = filename;
  }

  void CreateTestPLYFiles() {
    // Create simple PLY file manually since PLY format is simpler
    CreateTestPLYXYZ();
    CreateTestPLYXYZRGB();
    CreateInvalidPLY();
  }

  void CreateTestPLYXYZ() {
    std::string filename = test_dir_ / "test_xyz.ply";
    std::ofstream file(filename);
    
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "element vertex 3\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "end_header\n";
    file << "0.0 0.0 0.0\n";
    file << "1.0 1.0 1.0\n";
    file << "2.0 2.0 2.0\n";
    
    file.close();
    test_ply_xyz_ = filename;
  }

  void CreateTestPLYXYZRGB() {
    std::string filename = test_dir_ / "test_xyzrgb.ply";
    std::ofstream file(filename);
    
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "element vertex 3\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "property uchar red\n";
    file << "property uchar green\n";
    file << "property uchar blue\n";
    file << "end_header\n";
    file << "0.0 0.0 0.0 255 0 0\n";
    file << "1.0 1.0 1.0 0 255 0\n";
    file << "2.0 2.0 2.0 0 0 255\n";
    
    file.close();
    test_ply_xyzrgb_ = filename;
  }

  void CreateInvalidPLY() {
    std::string filename = test_dir_ / "invalid.ply";
    std::ofstream file(filename);
    file << "This is not a valid PLY file\n";
    file.close();
    test_invalid_ply_ = filename;
  }
#endif

  std::filesystem::path test_dir_;
  std::string test_pcd_xyz_;
  std::string test_pcd_xyzi_;
  std::string test_pcd_xyzrgb_;
  std::string test_invalid_pcd_;
  std::string test_ply_xyz_;
  std::string test_ply_xyzrgb_;
  std::string test_invalid_ply_;
};

#ifdef QUICKVIZ_WITH_PCL

TEST_F(PCLLoaderTest, DetectFormatFromExtension) {
  EXPECT_EQ(PointCloudLoader::DetectFormat("test.pcd"), PointCloudLoader::Format::kPCD);
  EXPECT_EQ(PointCloudLoader::DetectFormat("test.PCD"), PointCloudLoader::Format::kPCD);
  EXPECT_EQ(PointCloudLoader::DetectFormat("test.ply"), PointCloudLoader::Format::kPLY);
  EXPECT_EQ(PointCloudLoader::DetectFormat("test.PLY"), PointCloudLoader::Format::kPLY);
  
  EXPECT_THROW(PointCloudLoader::DetectFormat("test.txt"), UnsupportedFormatException);
  EXPECT_THROW(PointCloudLoader::DetectFormat("test"), UnsupportedFormatException);
}

TEST_F(PCLLoaderTest, IsFormatSupported) {
  EXPECT_TRUE(PointCloudLoader::IsFormatSupported(PointCloudLoader::Format::kPCD));
  EXPECT_TRUE(PointCloudLoader::IsFormatSupported(PointCloudLoader::Format::kPLY));
  EXPECT_TRUE(PointCloudLoader::IsFormatSupported(PointCloudLoader::Format::kAutoDetect));
}

TEST_F(PCLLoaderTest, GetSupportedExtensions) {
  auto extensions = PointCloudLoader::GetSupportedExtensions();
  EXPECT_EQ(extensions.size(), 2);
  EXPECT_NE(std::find(extensions.begin(), extensions.end(), ".pcd"), extensions.end());
  EXPECT_NE(std::find(extensions.begin(), extensions.end(), ".ply"), extensions.end());
}

TEST_F(PCLLoaderTest, AnalyzeFieldsPCDXYZ) {
  auto metadata = PointCloudLoader::AnalyzeFields(test_pcd_xyz_);
  
  EXPECT_EQ(metadata.format, "PCD");
  EXPECT_TRUE(metadata.fields.HasXYZ());
  EXPECT_FALSE(metadata.fields.has_intensity);
  EXPECT_FALSE(metadata.fields.HasRGBColor());
  EXPECT_FALSE(metadata.fields.HasRGBAColor());
  EXPECT_EQ(metadata.GetRecommendedPCLType(), "PointXYZ");
}

TEST_F(PCLLoaderTest, AnalyzeFieldsPCDXYZI) {
  auto metadata = PointCloudLoader::AnalyzeFields(test_pcd_xyzi_);
  
  EXPECT_EQ(metadata.format, "PCD");
  EXPECT_TRUE(metadata.fields.HasXYZ());
  EXPECT_TRUE(metadata.fields.has_intensity);
  EXPECT_FALSE(metadata.fields.HasRGBColor());
  EXPECT_EQ(metadata.GetRecommendedPCLType(), "PointXYZI");
}

TEST_F(PCLLoaderTest, AnalyzeFieldsPCDXYZRGB) {
  auto metadata = PointCloudLoader::AnalyzeFields(test_pcd_xyzrgb_);
  
  EXPECT_EQ(metadata.format, "PCD");
  EXPECT_TRUE(metadata.fields.HasXYZ());
  EXPECT_FALSE(metadata.fields.has_intensity);
  EXPECT_TRUE(metadata.fields.HasRGBColor());
  EXPECT_EQ(metadata.GetRecommendedPCLType(), "PointXYZRGB");
}

TEST_F(PCLLoaderTest, AnalyzeFieldsPLY) {
  auto metadata = PointCloudLoader::AnalyzeFields(test_ply_xyz_);
  
  EXPECT_EQ(metadata.format, "PLY");
  EXPECT_TRUE(metadata.fields.HasXYZ());
  EXPECT_FALSE(metadata.fields.has_intensity);
  EXPECT_FALSE(metadata.fields.HasRGBColor());
  EXPECT_EQ(metadata.GetRecommendedPCLType(), "PointXYZ");
}

TEST_F(PCLLoaderTest, AnalyzeFieldsPLYRGB) {
  auto metadata = PointCloudLoader::AnalyzeFields(test_ply_xyzrgb_);
  
  EXPECT_EQ(metadata.format, "PLY");
  EXPECT_TRUE(metadata.fields.HasXYZ());
  EXPECT_FALSE(metadata.fields.has_intensity);
  EXPECT_TRUE(metadata.fields.HasRGBColor());
  EXPECT_EQ(metadata.GetRecommendedPCLType(), "PointXYZRGB");
}

TEST_F(PCLLoaderTest, LoadPCDXYZToPCL) {
  auto [cloud, metadata] = PointCloudLoader::LoadToPCL<pcl::PointXYZ>(test_pcd_xyz_);
  
  EXPECT_NE(cloud, nullptr);
  EXPECT_EQ(cloud->points.size(), 100);
  EXPECT_EQ(metadata.point_count, 100);
  EXPECT_EQ(metadata.detected_pcl_type, "PointXYZ");
  EXPECT_TRUE(metadata.fields.HasXYZ());
  EXPECT_FALSE(metadata.fields.has_intensity);
}

TEST_F(PCLLoaderTest, LoadPCDXYZIToPCL) {
  auto [cloud, metadata] = PointCloudLoader::LoadToPCL<pcl::PointXYZI>(test_pcd_xyzi_);
  
  EXPECT_NE(cloud, nullptr);
  EXPECT_EQ(cloud->points.size(), 50);
  EXPECT_EQ(metadata.point_count, 50);
  EXPECT_EQ(metadata.detected_pcl_type, "PointXYZI");
  EXPECT_TRUE(metadata.fields.HasXYZ());
  EXPECT_TRUE(metadata.fields.has_intensity);
}

TEST_F(PCLLoaderTest, LoadPCDXYZRGBToPCL) {
  auto [cloud, metadata] = PointCloudLoader::LoadToPCL<pcl::PointXYZRGB>(test_pcd_xyzrgb_);
  
  EXPECT_NE(cloud, nullptr);
  EXPECT_EQ(cloud->points.size(), 75);
  EXPECT_EQ(metadata.point_count, 75);
  EXPECT_EQ(metadata.detected_pcl_type, "PointXYZRGB");
  EXPECT_TRUE(metadata.fields.HasXYZ());
  EXPECT_TRUE(metadata.fields.HasRGBColor());
}

// Disable renderer tests temporarily due to OpenGL context requirements
/*
TEST_F(PCLLoaderTest, LoadWithAutoDetectPCD) {
  PointCloud renderer_cloud;
  
  auto metadata = PointCloudLoader::Load(test_pcd_xyz_, renderer_cloud);
  
  EXPECT_EQ(metadata.format, "PCD");
  EXPECT_EQ(metadata.point_count, 100);
  EXPECT_EQ(metadata.detected_pcl_type, "PointXYZ");
}

TEST_F(PCLLoaderTest, LoadWithAutoDetectPLY) {
  PointCloud renderer_cloud;
  
  auto metadata = PointCloudLoader::Load(test_ply_xyz_, renderer_cloud);
  
  EXPECT_EQ(metadata.format, "PLY");
  EXPECT_EQ(metadata.point_count, 3);
  EXPECT_EQ(metadata.detected_pcl_type, "PointXYZ");
}

TEST_F(PCLLoaderTest, LoadWithProgressCallback) {
  PointCloud renderer_cloud;
  
  bool callback_called = false;
  float last_progress = 0.0f;
  std::string last_message;
  
  auto callback = [&](float progress, const std::string& message) {
    callback_called = true;
    last_progress = progress;
    last_message = message;
  };
  
  auto metadata = PointCloudLoader::LoadPCD(test_pcd_xyz_, renderer_cloud, callback);
  
  EXPECT_TRUE(callback_called);
  EXPECT_EQ(last_progress, 1.0f);
  EXPECT_EQ(last_message, "Loading complete");
}
*/

TEST_F(PCLLoaderTest, LoadToPCLTemplate) {
  auto [cloud, metadata] = PointCloudLoader::LoadToPCL<pcl::PointXYZ>(
      test_pcd_xyz_, PointCloudLoader::Format::kPCD);
  
  EXPECT_NE(cloud, nullptr);
  EXPECT_EQ(cloud->points.size(), 100);
  EXPECT_EQ(metadata.detected_pcl_type, "PointXYZ");
}

TEST_F(PCLLoaderTest, FileNotFoundError) {
  EXPECT_THROW(PointCloudLoader::LoadToPCL<pcl::PointXYZ>("nonexistent.pcd"), 
               FileNotFoundException);
}

TEST_F(PCLLoaderTest, InvalidFileError) {
  EXPECT_THROW(PointCloudLoader::LoadToPCL<pcl::PointXYZ>(test_invalid_pcd_), 
               CorruptedFileException);
}

TEST_F(PCLLoaderTest, UnsupportedFormatError) {
  EXPECT_THROW(PointCloudLoader::DetectFormat("test.txt"), 
               UnsupportedFormatException);
}

TEST_F(PCLLoaderTest, FieldDetectorPCDHeader) {
  auto fields = field_detector::ParsePCDHeader(test_pcd_xyzrgb_);
  
  EXPECT_TRUE(fields.HasXYZ());
  EXPECT_TRUE(fields.HasRGBColor());
  EXPECT_FALSE(fields.has_intensity);
}

TEST_F(PCLLoaderTest, FieldDetectorPLYHeader) {
  auto fields = field_detector::ParsePLYHeader(test_ply_xyzrgb_);
  
  EXPECT_TRUE(fields.HasXYZ());
  EXPECT_TRUE(fields.HasRGBColor());
  EXPECT_FALSE(fields.has_intensity);
}

TEST_F(PCLLoaderTest, FieldCompatibilityCheck) {
  PointCloudFields xyz_fields;
  xyz_fields.has_x = xyz_fields.has_y = xyz_fields.has_z = true;
  
  PointCloudFields xyzi_fields = xyz_fields;
  xyzi_fields.has_intensity = true;
  
  PointCloudFields xyzrgb_fields = xyz_fields;
  xyzrgb_fields.has_r = xyzrgb_fields.has_g = xyzrgb_fields.has_b = true;
  
  EXPECT_TRUE(field_detector::IsCompatiblePCLType<pcl::PointXYZ>(xyz_fields));
  EXPECT_TRUE(field_detector::IsCompatiblePCLType<pcl::PointXYZI>(xyzi_fields));
  EXPECT_TRUE(field_detector::IsCompatiblePCLType<pcl::PointXYZRGB>(xyzrgb_fields));
  
  EXPECT_FALSE(field_detector::IsCompatiblePCLType<pcl::PointXYZI>(xyz_fields));
  EXPECT_FALSE(field_detector::IsCompatiblePCLType<pcl::PointXYZRGB>(xyz_fields));
}

TEST_F(PCLLoaderTest, RequiredFieldsForPCLTypes) {
  auto xyz_fields = field_detector::GetRequiredFields("PointXYZ");
  EXPECT_TRUE(xyz_fields.HasXYZ());
  EXPECT_FALSE(xyz_fields.has_intensity);
  
  auto xyzi_fields = field_detector::GetRequiredFields("PointXYZI");
  EXPECT_TRUE(xyzi_fields.HasXYZ());
  EXPECT_TRUE(xyzi_fields.has_intensity);
  
  auto xyzrgb_fields = field_detector::GetRequiredFields("PointXYZRGB");
  EXPECT_TRUE(xyzrgb_fields.HasXYZ());
  EXPECT_TRUE(xyzrgb_fields.HasRGBColor());
}

#else // QUICKVIZ_WITH_PCL

TEST_F(PCLLoaderTest, PCLNotAvailable) {
  // When PCL is not available, we should still be able to compile
  // but functionality will be limited
  GTEST_SKIP() << "PCL not available, skipping PCL loader tests";
}

#endif // QUICKVIZ_WITH_PCL

} // namespace test
} // namespace pcl_bridge
} // namespace quickviz