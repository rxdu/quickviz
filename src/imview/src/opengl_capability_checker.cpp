/*
 * opengl_capability_checker.cpp
 *
 * Created on: 2025-01-09
 * Description: OpenGL capability detection and validation utility
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "imview/opengl_capability_checker.hpp"

#include <iostream>
#include <sstream>
#include <algorithm>
#include <cstring>

#ifdef IMVIEW_WITH_GLAD
#include "glad/glad.h"
#else
#include <GL/gl.h>
#endif

namespace quickviz {

OpenGLCapabilities OpenGLCapabilityChecker::CheckCapabilities(int required_major, 
                                                             int required_minor) {
  OpenGLCapabilities caps;
  
  try {
    // Get OpenGL version string
    const char* version_str = reinterpret_cast<const char*>(glGetString(GL_VERSION));
    if (version_str) {
      caps.version_string = version_str;
    } else {
      caps.error_message = "Failed to get OpenGL version string";
      return caps;
    }

    // Get vendor information
    const char* vendor_str = reinterpret_cast<const char*>(glGetString(GL_VENDOR));
    if (vendor_str) {
      caps.vendor = vendor_str;
    }

    // Get renderer information
    const char* renderer_str = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
    if (renderer_str) {
      caps.renderer = renderer_str;
    }

    // Get GLSL version
    const char* glsl_version_str = reinterpret_cast<const char*>(glGetString(GL_SHADING_LANGUAGE_VERSION));
    if (glsl_version_str) {
      caps.glsl_version = glsl_version_str;
    }

    // Parse OpenGL version numbers
    if (sscanf(version_str, "%d.%d", &caps.major_version, &caps.minor_version) != 2) {
      // Try alternative parsing for some drivers
      if (sscanf(version_str, "OpenGL ES %d.%d", &caps.major_version, &caps.minor_version) != 2) {
        caps.error_message = "Failed to parse OpenGL version numbers from: " + caps.version_string;
        return caps;
      }
    }

    // Check if we have the required OpenGL version
    if (caps.major_version > required_major || 
        (caps.major_version == required_major && caps.minor_version >= required_minor)) {
      caps.supports_required_version = true;
    } else {
      std::stringstream ss;
      ss << "OpenGL " << required_major << "." << required_minor 
         << " required, but only " << caps.major_version << "." << caps.minor_version 
         << " is available";
      caps.error_message = ss.str();
    }

    // Check for core profile (OpenGL 3.2+)
    if (caps.major_version >= 3 && caps.minor_version >= 2) {
      GLint profile = 0;
      glGetIntegerv(GL_CONTEXT_PROFILE_MASK, &profile);
      caps.core_profile = (profile & GL_CONTEXT_CORE_PROFILE_BIT) != 0;
    }

    // Get available extensions
    caps.available_extensions = GetAvailableExtensions();

    // Check for common problematic configurations
    if (IsIntelGPU(caps.vendor, caps.renderer)) {
      // Intel GPUs sometimes have issues with OpenGL 3.3+ on older drivers
      if (caps.major_version == 3 && caps.minor_version >= 3) {
        std::cout << "WARNING: Intel GPU detected. If you experience shader issues, "
                  << "consider updating your graphics drivers." << std::endl;
      }
    }

    // Check for Mesa software rendering
    if (caps.renderer.find("llvmpipe") != std::string::npos || 
        caps.renderer.find("softpipe") != std::string::npos) {
      std::cout << "WARNING: Software rendering detected (" << caps.renderer 
                << "). Performance may be poor." << std::endl;
    }

  } catch (const std::exception& e) {
    caps.error_message = "Exception during OpenGL capability check: " + std::string(e.what());
  }

  return caps;
}

void OpenGLCapabilityChecker::PrintCapabilities(const OpenGLCapabilities& capabilities) {
  std::cout << "=== OpenGL Capabilities ===" << std::endl;
  std::cout << "Version: " << capabilities.version_string << std::endl;
  std::cout << "Vendor: " << capabilities.vendor << std::endl;
  std::cout << "Renderer: " << capabilities.renderer << std::endl;
  std::cout << "GLSL Version: " << capabilities.glsl_version << std::endl;
  std::cout << "Parsed Version: " << capabilities.major_version << "." << capabilities.minor_version << std::endl;
  std::cout << "Core Profile: " << (capabilities.core_profile ? "Yes" : "No") << std::endl;
  std::cout << "Required Version Support: " << (capabilities.supports_required_version ? "Yes" : "No") << std::endl;
  
  if (!capabilities.error_message.empty()) {
    std::cout << "Error: " << capabilities.error_message << std::endl;
  }
  
  std::cout << "Available Extensions: " << capabilities.available_extensions.size() << std::endl;
  std::cout << "=========================" << std::endl;
}

bool OpenGLCapabilityChecker::IsExtensionSupported(const std::string& extension_name) {
  auto extensions = GetAvailableExtensions();
  return std::find(extensions.begin(), extensions.end(), extension_name) != extensions.end();
}

bool OpenGLCapabilityChecker::ValidateRequiredFeatures(const OpenGLCapabilities& capabilities) {
  if (!capabilities.supports_required_version) {
    return false;
  }

  // Check for essential OpenGL 3.3 features
  if (capabilities.major_version >= 3 && capabilities.minor_version >= 3) {
    // Vertex Array Objects should be available
    if (!IsExtensionSupported("GL_ARB_vertex_array_object") && 
        !(capabilities.major_version > 3 || (capabilities.major_version == 3 && capabilities.minor_version >= 3))) {
      std::cout << "WARNING: Vertex Array Objects may not be supported" << std::endl;
    }
  }

  return true;
}

std::vector<std::string> OpenGLCapabilityChecker::GetAvailableExtensions() {
  std::vector<std::string> extensions;
  
  // Try modern way first (OpenGL 3.0+)
  GLint num_extensions = 0;
  glGetIntegerv(GL_NUM_EXTENSIONS, &num_extensions);
  
  if (num_extensions > 0) {
    for (GLint i = 0; i < num_extensions; i++) {
      const char* ext = reinterpret_cast<const char*>(glGetStringi(GL_EXTENSIONS, i));
      if (ext) {
        extensions.push_back(ext);
      }
    }
  } else {
    // Fallback to legacy method (OpenGL < 3.0)
    const char* extensions_str = reinterpret_cast<const char*>(glGetString(GL_EXTENSIONS));
    if (extensions_str) {
      std::string ext_string(extensions_str);
      std::istringstream iss(ext_string);
      std::string ext;
      while (iss >> ext) {
        extensions.push_back(ext);
      }
    }
  }
  
  return extensions;
}

bool OpenGLCapabilityChecker::IsIntelGPU(const std::string& vendor, const std::string& renderer) {
  std::string vendor_lower = vendor;
  std::string renderer_lower = renderer;
  std::transform(vendor_lower.begin(), vendor_lower.end(), vendor_lower.begin(), ::tolower);
  std::transform(renderer_lower.begin(), renderer_lower.end(), renderer_lower.begin(), ::tolower);
  
  return vendor_lower.find("intel") != std::string::npos ||
         renderer_lower.find("intel") != std::string::npos;
}

bool OpenGLCapabilityChecker::IsAMDGPU(const std::string& vendor, const std::string& renderer) {
  std::string vendor_lower = vendor;
  std::string renderer_lower = renderer;
  std::transform(vendor_lower.begin(), vendor_lower.end(), vendor_lower.begin(), ::tolower);
  std::transform(renderer_lower.begin(), renderer_lower.end(), renderer_lower.begin(), ::tolower);
  
  return vendor_lower.find("amd") != std::string::npos ||
         vendor_lower.find("ati") != std::string::npos ||
         renderer_lower.find("radeon") != std::string::npos;
}

bool OpenGLCapabilityChecker::IsNVIDIAGPU(const std::string& vendor, const std::string& renderer) {
  std::string vendor_lower = vendor;
  std::string renderer_lower = renderer;
  std::transform(vendor_lower.begin(), vendor_lower.end(), vendor_lower.begin(), ::tolower);
  std::transform(renderer_lower.begin(), renderer_lower.end(), renderer_lower.begin(), ::tolower);
  
  return vendor_lower.find("nvidia") != std::string::npos ||
         renderer_lower.find("geforce") != std::string::npos ||
         renderer_lower.find("quadro") != std::string::npos;
}

} // namespace quickviz