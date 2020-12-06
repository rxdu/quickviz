/*
 * test_data_plotter.cpp
 *
 * Created on: Dec 06, 2020 00:06
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include <math.h>

#include <memory>

#include "plotter/data_plotter.hpp"

using namespace rdu;

std::shared_ptr<DataPlotter> plotter;

// struct RTPlotter : public DataPlotter {
//   void Draw() override {

//   }
// };

int main(int argc, const char* argv[]) {
  plotter = std::make_shared<DataPlotter>();
  plotter->Show();
  return 0;
}
