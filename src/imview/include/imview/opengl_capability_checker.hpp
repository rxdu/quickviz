/*
 * opengl_capability_checker.hpp
 *
 * Created on: 2025-01-09
 * Description: OpenGL capability detection and validation utility
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#pragma once

#include <string>
#include <vector>

namespace quickviz {

struct OpenGLCapabilities {
  int major_version = 0;
  int minor_version = 0;
  std::string version_string;
  std::string vendor;
  std::string renderer;
  std::string glsl_version;
  bool core_profile = false;
  bool supports_required_version = false;
  std::vector<std::string> available_extensions;
  std::string error_message;
};

class OpenGLCapabilityChecker {
public:
  /**
   * @brief Check OpenGL capabilities and validate minimum requirements
   * @param required_major Minimum required OpenGL major version (default: 3)
   * @param required_minor Minimum required OpenGL minor version (default: 3)
   * @return OpenGLCapabilities structure with detected capabilities
   */
  static OpenGLCapabilities CheckCapabilities(int required_major = 3, 
                                            int required_minor = 3);

  /**
   * @brief Print detailed OpenGL capability information to console
   * @param capabilities OpenGL capabilities structure
   */
  static void PrintCapabilities(const OpenGLCapabilities& capabilities);

  /**
   * @brief Check if a specific OpenGL extension is supported
   * @param extension_name Name of the extension to check
   * @return True if extension is supported, false otherwise
   */
  static bool IsExtensionSupported(const std::string& extension_name);

  /**
   * @brief Validate that the OpenGL context supports required features
   * @param capabilities OpenGL capabilities structure
   * @return True if all required features are supported, false otherwise
   */
  static bool ValidateRequiredFeatures(const OpenGLCapabilities& capabilities);

private:
  static std::vector<std::string> GetAvailableExtensions();
  static bool IsIntelGPU(const std::string& vendor, const std::string& renderer);
  static bool IsAMDGPU(const std::string& vendor, const std::string& renderer);
  static bool IsNVIDIAGPU(const std::string& vendor, const std::string& renderer);
};

} // namespace quickviz