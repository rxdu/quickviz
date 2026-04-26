/*
 * @file config_panel.hpp
 * @date 10/8/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_CONFIG_PANEL_HPP
#define QUICKVIZ_CONFIG_PANEL_HPP

#include "imview/panel.hpp"

namespace quickviz {
class ConfigPanel : public Panel {
 public:
  ConfigPanel(std::string name = "Config");

  void Draw() override;
};
}  // namespace quickviz

#endif  // QUICKVIZ_CONFIG_PANEL_HPP