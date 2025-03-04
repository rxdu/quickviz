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

#include "imview/terminal/tui_viewer.hpp"

namespace quickviz {
TuiViewer::TuiViewer(const std::string &title, bool has_border)
    : title_(title), has_border_(has_border) {
  log_file_.open("/tmp/tui_viewer.log", std::ios::out | std::ios::app);

  Init();
  Resize();
}

TuiViewer::~TuiViewer() {
  log_file_.close();
  Deinit();
}

bool TuiViewer::AddSceneObject(std::shared_ptr<SceneObject> obj) {
  if (obj == nullptr) {
    std::cerr << "[ERROR] TuiViewer::AddSceneObject(): object is nullptr"
              << std::endl;
    return false;
  }
  scene_objects_.push_back(obj);
  return true;
}

void TuiViewer::Init() {
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

  TuiText::InitColor();
}

void TuiViewer::Deinit() { endwin(); }

void TuiViewer::Resize() {
  Deinit();
  Init();
  getmaxyx(stdscr, term_size_y_, term_size_x_);
  for (auto &obj : scene_objects_) {
    obj->OnResize(term_size_x_, term_size_y_);
  }
  log_file_ << "term_size_y_: " << term_size_y_
            << ", term_size_x_: " << term_size_x_ << std::endl;
}

void TuiViewer::ForceRefresh() {
  Resize();
  for (auto &panel : scene_objects_) {
    panel->OnRender();
  }
}

void TuiViewer::Show() {
  ForceRefresh();
  while (true) {
    // handle input
    int input_ch = getch();
    switch (input_ch) {
      case KEY_RESIZE: {
        Resize();
        break;
      }
    }

    // render border if needed
    // if (has_border_) {
    //   box(stdscr, 0, 0);
    // }

    // render panels
    for (auto &panel : scene_objects_) {
      panel->OnRender();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
}  // namespace quickviz
