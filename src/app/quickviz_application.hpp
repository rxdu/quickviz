/*
 * @file quickviz_application.hpp
 * @date 10/8/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_QUICKVIZ_APPLICATION_HPP
#define QUICKVIZ_QUICKVIZ_APPLICATION_HPP

#include "imview/viewer.hpp"
#include "imview/layer.hpp"
#include "panels/main_docking_panel.hpp"
#include "data_reader.hpp"

namespace quickviz {
class QuickvizApplication {
 public:
  QuickvizApplication();
  ~QuickvizApplication() = default;

  // do not allow copy
  QuickvizApplication(const QuickvizApplication &) = delete;
  QuickvizApplication &operator=(const QuickvizApplication &) = delete;

  // Run the application
  bool Initialize();
  void Run();

 private:
  std::unique_ptr<Viewer> viewer_;

  // ui elements
  std::shared_ptr<Layer> ui_layer_;
  std::shared_ptr<MainDockingPanel> main_docking_panel_;

  std::shared_ptr<Layer> plot_layer_;

  // data reader
  std::shared_ptr<DataReader> data_reader_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_QUICKVIZ_APPLICATION_HPP