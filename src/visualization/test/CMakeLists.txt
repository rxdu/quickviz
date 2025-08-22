# Silence PCL-era policy warnings, but keep modern behavior where safe
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()
find_package(PCL QUIET COMPONENTS common io search segmentation)
if(PCL_FOUND)
  message(STATUS "Found PCL: ${PCL_VERSION}")

  add_executable(test_pcd test_pcd.cpp)
  target_include_directories(test_pcd PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_pcd PRIVATE gldraw ${PCL_LIBRARIES})
  target_compile_definitions(test_pcd PRIVATE ${PCL_DEFINITIONS})
  
  add_executable(test_pcl_bridge test_pcl_bridge.cpp)
  target_include_directories(test_pcl_bridge PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_pcl_bridge PRIVATE gldraw visualization ${PCL_LIBRARIES})
  target_compile_definitions(test_pcl_bridge PRIVATE ${PCL_DEFINITIONS})
  
  add_executable(test_pcd_with_selection test_pcd_with_selection.cpp)
  target_include_directories(test_pcd_with_selection PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_pcd_with_selection PRIVATE gldraw visualization ${PCL_LIBRARIES})
  target_compile_definitions(test_pcd_with_selection PRIVATE ${PCL_DEFINITIONS})
  
#  add_executable(test_pcl_loader unit_test/test_pcl_loader.cpp)
#  target_include_directories(test_pcl_loader PRIVATE ${PCL_INCLUDE_DIRS})
#  target_link_libraries(test_pcl_loader PRIVATE gldraw ${PCL_LIBRARIES} gtest_main)
#  target_compile_definitions(test_pcl_loader PRIVATE ${PCL_DEFINITIONS})
  
  add_executable(test_pcl_loader_render test_pcl_loader_render.cpp)
  target_include_directories(test_pcl_loader_render PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_pcl_loader_render PRIVATE gldraw visualization ${PCL_LIBRARIES})
  target_compile_definitions(test_pcl_loader_render PRIVATE ${PCL_DEFINITIONS})

  add_subdirectory(unit_test)
endif()
