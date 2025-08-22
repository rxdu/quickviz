/*
 * @file point_cloud_info_panel.hpp
 * @date 8/21/25
 * @brief
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_POINT_CLOUD_INFO_PANEL_HPP
#define QUICKVIZ_POINT_CLOUD_INFO_PANEL_HPP

#include "imview/panel.hpp"
#include "visualization/pcl_bridge/pcl_loader.hpp"

namespace quickviz {
class PointCloudInfoPanel : public quickviz::Panel {
 public:
  PointCloudInfoPanel(const std::string& name,
                      const pcl_bridge::PointCloudMetadata& metadata)
      : quickviz::Panel(name), metadata_(metadata) {}

  void Draw() override;

 private:
  pcl_bridge::PointCloudMetadata metadata_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_POINT_CLOUD_INFO_PANEL_HPP