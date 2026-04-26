# Core + ImView Code Quality Recommendations

Date: 2025-09-01

This document captures targeted, high‑impact improvements for the `core` and `viewer` modules. Items are grouped by module with concrete actions and file pointers.

## Core

### BufferRegistry: remove singleton, strengthen typing
- Problem: Global singleton and `dynamic_pointer_cast` on `BufferInterface<T>` create hidden dependencies and runtime type errors.
  - Files: `src/core/include/core/buffer/buffer_registry.hpp`, `src/core/src/buffer/buffer_registry.cpp`
- Improve:
  - Introduce `IBufferRegistry` interface and a concrete `DefaultBufferRegistry`; inject instances at app/window scope.
  - Track stored type with `std::type_index` alongside the pointer (clear mismatch diagnostics), or use `std::any` with checked accessors.
  - Replace "check then assign" with `emplace` and return `bool/std::optional` for recoverable states (reserve exceptions for programming errors).

### ThreadSafeQueue: correct move, add shutdown protocol
- Problems:
  - Move constructor implementation is incorrect (self-move bug, missing lock on source).
  - No cooperative shutdown; `Pop()` can block indefinitely.
  - File: `src/core/include/core/event/thread_safe_queue.hpp`
- Improve:
  - Implement move ctor/assign using `std::scoped_lock` on both mutexes; move underlying queue.
  - Add `Close()`/`IsClosed()` flags; make `Pop()` return optional or throw `queue_closed` to unblock gracefully.
  - Consider `WaitPop(T&, std::stop_token)` overloads for cancellation.

### AsyncEventDispatcher: instance-based, robust threading
- Problems: Hard thread coupling via first-caller thread IDs; throws if `Dispatch`/`HandleEvents` are called on unexpected threads.
  - Files: `src/core/include/core/event/async_event_dispatcher.hpp`, `src/core/src/event/async_event_dispatcher.cpp`
- Improve:
  - Convert to instance-based with owned worker thread: `Start()`, `Stop()`, RAII lifecycle; drain queue with blocking `Pop()`.
  - Unify handler signatures to return `bool` (consumption) and optionally support priority sorting.
  - Use handler tokens for deregistration; avoid copying handler vectors per event (snapshot under mutex then iterate).

### Event APIs: semantics, const-correctness
- Align sync/async dispatchers to consistent consumption semantics (`bool` → consumed).
- Make read-only methods `const`, mark trivial dtors `= default`, add `[[nodiscard]]` to results that should not be ignored.

### Fonts/resources: limit header bloat
- Problem: Large embedded fonts in public headers increase compile time.
  - Files: `src/core/include/core/fonts/*`, generator `src/core/src/fonts/binary_to_compressed_c.cpp`
- Improve: Move font binaries behind `.cpp` or to `resources/` and load at runtime; keep public headers lightweight.

### Error handling/logging
- Prefer `std::optional/std::expected`-style APIs for recoverable states in registries over exceptions.
- Replace `std::cerr` with a lightweight logger (levels, compile-time toggles) across core.

## ImView

### Centralize input: single InputManager per Window
- Problem: `Panel` keeps its own `InputManager` and polls ImGui state; `Window` also owns an `InputManager`.
  - Files: `src/viewer/include/viewer/panel.hpp`, `src/viewer/src/panel.cpp`, `src/viewer/include/viewer/window.hpp`, `src/viewer/src/window.cpp`
- Improve:
  - Panels should register as `InputEventHandler`s on the Window’s single `InputManager`; provide `AttachTo(Window&)` or inject via ctor.
  - Poll ImGui once per frame at the Window and dispatch through the centralized `InputDispatcher`.

### ImGui capture and debug output
- Problem: Hidden Ctrl+Shift+K bypass and periodic `std::cout` noise in release.
  - File: `src/viewer/src/input/imgui_input_utils.cpp`
- Improve: Gate with `#ifdef QUICKVIZ_INPUT_DEBUG` or a runtime flag; route through logger. Make bypass behavior configurable via `InputManager`/Window settings.

### GamepadManager: state ownership, singleton removal path
- Problems:
  - `GamepadManager` is a singleton; `ImGuiInputUtils` uses a static `previous_states` map outside the manager.
  - Files: `src/viewer/include/viewer/input/gamepad_manager.hpp`, `src/viewer/src/input/gamepad_manager.cpp`, `src/viewer/src/input/imgui_input_utils.cpp`
- Improve:
  - Move previous-state tracking into `GamepadManager` per device and expose delta-friendly queries.
  - Long term: make `GamepadManager` instance-owned by `Window` (align with `InputManager`).

### Panel ergonomics: reduce flag boilerplate, add RAII helpers
- Replace many `SetNoXxx(bool)` calls with presets (e.g., overlay/panel/fullscreen) or a direct `SetFlags(ImGuiWindowFlags)`.
- Add RAII wrappers for ImGui/ImPlot Push/Pop sequences to prevent mismatches (see `src/widget/src/rt_line_plot_widget.cpp`).

### Window lifecycle: GLFW ownership
- Problem: `Window::~Window()` calls `glfwTerminate()` unconditionally; breaks multi-window use.
  - File: `src/viewer/src/window.cpp`
- Improve: Centralize GLFW init/term in an `Application`-level owner or use ref-counting; windows only destroy their own contexts.

### CMake options and link scopes
- Add `option(ENABLE_AUTO_LAYOUT ...)`, `option(ENABLE_TUI_SUPPORT ...)`, `option(VIEWER_WITH_GLAD ...)` with defaults and help strings.
- Use proper scope for compile definitions and link interfaces; minimize PUBLIC exposure.
  - Files: `src/viewer/CMakeLists.txt`, `src/core/CMakeLists.txt`

## Cross‑Cutting

### Modern C++ practices
- Prefer `std::scoped_lock` for multi-mutex locking; use `noexcept` where appropriate.
- Add `[[nodiscard]]` for functions where ignoring the result is a bug (e.g., registry lookups, dispatcher registration returning tokens).
- Normalize include guards (`QUICKVIZ_*`), fix legacy names (e.g., guard in `panel.hpp`).

### Testing and tooling
- Add tests:
  - `ThreadSafeQueue` move + shutdown behavior.
  - `AsyncEventDispatcher` start/stop, handler consumption ordering.
  - Input dispatch ordering by priority and panel input policies.
- Enable sanitizers in CI for tests (`-fsanitize=address,undefined` on non-MSVC debug builds) and keep frame pointers.
- Optional: introduce `.clang-tidy` with `modernize-*`, `bugprone-*`, `readability-*`.

### Documentation
- Add brief Doxygen-style usage notes to key headers clarifying ownership and threading (e.g., who owns managers, whether an API is thread-safe).
- Keep module-level design notes updated (this doc plus `docs/notes/*`).

## Immediate Action Items (Prioritized)
1) Fix `ThreadSafeQueue` move + add `Close()` to unblock `Pop()`.
2) Move gamepad previous-state tracking into `GamepadManager` (remove static map in `ImGuiInputUtils`).
3) Deprecate `BufferRegistry::GetInstance()` and introduce `IBufferRegistry`; start injecting into widgets (e.g., `rt_line_plot_widget`).
4) Refactor `AsyncEventDispatcher` to instance-based worker with clean lifecycle.
5) Centralize input: one `InputManager` per Window; Panels register as handlers; Window polls once.

## Notes
- Related references in-tree:
  - BufferRegistry usages: search for `BufferRegistry::GetInstance()` across `widget/` and `gldraw/`.
  - Existing design notes: `docs/notes/core_module_review_2025-01-28.md`.

