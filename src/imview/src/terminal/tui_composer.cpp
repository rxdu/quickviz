/**
 * @file tui_composer.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-02-27
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <stdexcept>
#include <thread>
#include <chrono>
#include <iostream>

#include "imview/terminal/tui_composer.hpp"

namespace quickviz {
TuiComposer::TuiComposer(const std::string &title, bool has_border)
    : title_(title), has_border_(has_border) {
  if (!initscr()) {
    throw std::runtime_error("Failed to initialize ncurses");
  }

  refresh();
  cbreak();
  noecho();
  nonl();
  curs_set(FALSE);
  intrflush(stdscr, FALSE);
  keypad(stdscr, TRUE);

  Resize();
}

TuiComposer::~TuiComposer() { endwin(); }

bool TuiComposer::AddSceneObject(std::shared_ptr<SceneObject> obj) {
  if (obj == nullptr) {
    std::cerr << "[ERROR] TuiComposer::AddSceneObject(): object is nullptr"
              << std::endl;
    return false;
  }
  scene_objects_.push_back(obj);
  return true;
}

void TuiComposer::Resize() {
  getmaxyx(stdscr, term_size_y_, term_size_x_);
  for (auto &panel : scene_objects_) {
    panel->OnResize(term_size_y_, term_size_x_);
  }
}

void TuiComposer::Show() {
  while (true) {
    // handle resize event
    if (term_size_y_ != getmaxy(stdscr) || term_size_x_ != getmaxx(stdscr)) {
      Resize();
    }

    // render border if needed
    if (has_border_) {
      box(stdscr, 0, 0);
    }

    // render panels
    for (auto &panel : scene_objects_) {
      panel->OnRender();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
}  // namespace quickviz
