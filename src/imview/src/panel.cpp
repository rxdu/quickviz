/*
 * panel.cpp
 *
 * Created on 4/3/22 11:07 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "imview/panel.hpp"

namespace quickviz {
Panel::Panel(std::string name) : SceneObject(name) {}

void Panel::OnRender() {
  if (auto_layout_) {
    ImGui::SetNextWindowSize(ImVec2(width_, height_));
    ImGui::SetNextWindowPos(ImVec2(x_, y_));
  }
  Draw();
}

void Panel::Begin(bool *p_open) { ImGui::Begin(name_.c_str(), p_open, flags_); }

void Panel::End() { ImGui::End(); }

void Panel::SetAutoLayout(bool value) { auto_layout_ = value; }

void Panel::SetNoTitleBar(bool value) {
  if (value) {
    flags_ |= ImGuiWindowFlags_NoTitleBar;
  } else {
    flags_ &= ~ImGuiWindowFlags_NoTitleBar;
  }
}

void Panel::SetNoResize(bool value) {
  if (value) {
    flags_ |= ImGuiWindowFlags_NoResize;
  } else {
    flags_ &= ~ImGuiWindowFlags_NoResize;
  }
}

void Panel::SetNoMove(bool value) {
  if (value) {
    flags_ |= ImGuiWindowFlags_NoMove;
  } else {
    flags_ &= ~ImGuiWindowFlags_NoMove;
  }
}

void Panel::SetNoScrollbar(bool value) {
  if (value) {
    flags_ |= ImGuiWindowFlags_NoScrollbar;
  } else {
    flags_ &= ~ImGuiWindowFlags_NoScrollbar;
  }
}

void Panel::SetNoScrollWithMouse(bool value) {
  if (value) {
    flags_ |= ImGuiWindowFlags_NoScrollWithMouse;
  } else {
    flags_ &= ~ImGuiWindowFlags_NoScrollWithMouse;
  }
}

void Panel::SetNoCollapse(bool value) {
  if (value) {
    flags_ |= ImGuiWindowFlags_NoCollapse;
  } else {
    flags_ &= ~ImGuiWindowFlags_NoCollapse;
  }
}

void Panel::SetAlwaysAutoResize(bool value) {
  if (value) {
    flags_ |= ImGuiWindowFlags_AlwaysAutoResize;
  } else {
    flags_ &= ~ImGuiWindowFlags_AlwaysAutoResize;
  }
}

void Panel::SetNoBackground(bool value) {
  if (value) {
    flags_ |= ImGuiWindowFlags_NoBackground;
  } else {
    flags_ &= ~ImGuiWindowFlags_NoBackground;
  }
}

void Panel::SetNoSavedSettings(bool value) {
  if (value) {
    flags_ |= ImGuiWindowFlags_NoSavedSettings;
  } else {
    flags_ &= ~ImGuiWindowFlags_NoSavedSettings;
  }
}

void Panel::SetNoMouseInputs(bool value) {
  if (value) {
    flags_ |= ImGuiWindowFlags_NoMouseInputs;
  } else {
    flags_ &= ~ImGuiWindowFlags_NoMouseInputs;
  }
}

void Panel::SetMenuBar(bool value) {
  if (value) {
    flags_ |= ImGuiWindowFlags_MenuBar;
  } else {
    flags_ &= ~ImGuiWindowFlags_MenuBar;
  }
}

void Panel::SetHorizontalScrollbar(bool value) {
  if (value) {
    flags_ |= ImGuiWindowFlags_HorizontalScrollbar;
  } else {
    flags_ &= ~ImGuiWindowFlags_HorizontalScrollbar;
  }
}

void Panel::SetNoFocusOnAppearing(bool value) {
  if (value) {
    flags_ |= ImGuiWindowFlags_NoFocusOnAppearing;
  } else {
    flags_ &= ~ImGuiWindowFlags_NoFocusOnAppearing;
  }
}

void Panel::SetNoBringToFrontOnFocus(bool value) {
  if (value) {
    flags_ |= ImGuiWindowFlags_NoBringToFrontOnFocus;
  } else {
    flags_ &= ~ImGuiWindowFlags_NoBringToFrontOnFocus;
  }
}

void Panel::SetAlwaysVerticalScrollbar(bool value) {
  if (value) {
    flags_ |= ImGuiWindowFlags_AlwaysVerticalScrollbar;
  } else {
    flags_ &= ~ImGuiWindowFlags_AlwaysVerticalScrollbar;
  }
}

void Panel::SetAlwaysHorizontalScrollbar(bool value) {
  if (value) {
    flags_ |= ImGuiWindowFlags_AlwaysHorizontalScrollbar;
  } else {
    flags_ &= ~ImGuiWindowFlags_AlwaysHorizontalScrollbar;
  }
}

void Panel::SetNoNavInputs(bool value) {
  if (value) {
    flags_ |= ImGuiWindowFlags_NoNavInputs;
  } else {
    flags_ &= ~ImGuiWindowFlags_NoNavInputs;
  }
}

void Panel::SetNoNavFocus(bool value) {
  if (value) {
    flags_ |= ImGuiWindowFlags_NoNavFocus;
  } else {
    flags_ &= ~ImGuiWindowFlags_NoNavFocus;
  }
}

void Panel::SetUnsavedDocument(bool value) {
  if (value) {
    flags_ |= ImGuiWindowFlags_UnsavedDocument;
  } else {
    flags_ &= ~ImGuiWindowFlags_UnsavedDocument;
  }
}

void Panel::SetNoDocking(bool value) {
  if (value) {
    flags_ |= ImGuiWindowFlags_NoDocking;
  } else {
    flags_ &= ~ImGuiWindowFlags_NoDocking;
  }
}

void Panel::SetNoNav() { flags_ |= ImGuiWindowFlags_NoNav; }

void Panel::SetNoDecoration() { flags_ |= ImGuiWindowFlags_NoDecoration; }

void Panel::SetNoInputs() { flags_ |= ImGuiWindowFlags_NoInputs; }
}  // namespace quickviz