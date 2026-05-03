# Library modules — visualization-first.
#
# Module layout (one job each):
#   core         — events, buffers, threading helpers
#   viewer       — window + panels + layout
#   scene        — interactive 3D scene rendering (OpenGL)
#   plot         — data charts (ImPlot 2D + ImPlot3D)
#   canvas       — 2D vector drawing (Cairo)
#   image        — image display & annotation (OpenCV, optional)
#   pcl_bridge   — PCL adapter
#
# Editor / undo / project-files / app frameworks are NOT in scope here;
# they live in sample/ on top of this library. CI enforces the boundary.
add_subdirectory(core)
add_subdirectory(viewer)
add_subdirectory(scene)
add_subdirectory(plot)
add_subdirectory(canvas)
add_subdirectory(image)  # internally returns early if OpenCV is missing
add_subdirectory(pcl_bridge)
