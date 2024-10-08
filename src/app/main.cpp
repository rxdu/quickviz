/*
 * @file app_quickviz.cpp
 * @date 9/29/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>

#include "imview/viewer.hpp"

#include "quickviz_application.hpp"

using namespace quickviz;

int main(int argc, char *argv[]) {
  QuickvizApplication app;
  if (app.Initialize()) app.Run();
  return 0;
}