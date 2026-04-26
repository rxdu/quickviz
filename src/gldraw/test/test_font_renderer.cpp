/**
 * @file test_font_renderer.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-29
 * @brief Test for FontRenderer to verify font loading and text metrics
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "gldraw/font_renderer.hpp"

using namespace quickviz;

class MinimalOpenGLContext {
public:
    MinimalOpenGLContext() : window_(nullptr) {}
    
    ~MinimalOpenGLContext() {
        if (window_) {
            glfwTerminate();
        }
    }
    
    bool Initialize() {
        // Initialize GLFW
        if (!glfwInit()) {
            std::cerr << "Failed to initialize GLFW" << std::endl;
            return false;
        }
        
        // Set OpenGL context hints
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE); // Show window for visual test
        
        // Create window
        window_ = glfwCreateWindow(800, 600, "FontRenderer Test", nullptr, nullptr);
        if (!window_) {
            std::cerr << "Failed to create GLFW window" << std::endl;
            glfwTerminate();
            return false;
        }
        
        glfwMakeContextCurrent(window_);
        
        // Initialize GLAD
        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
            std::cerr << "Failed to initialize GLAD" << std::endl;
            return false;
        }
        
        std::cout << "✓ OpenGL Context created: " << glGetString(GL_VERSION) << std::endl;
        return true;
    }
    
    GLFWwindow* GetWindow() const { return window_; }

private:
    GLFWwindow* window_;
};

void TestFontFileAccess() {
  std::cout << "=== Font File Access Test ===\n";
  
  // Test if font file exists and is readable
  std::ifstream font_file("../assets/fonts/OpenSans-Bold.ttf", std::ios::binary | std::ios::ate);
  if (font_file.is_open()) {
    size_t file_size = font_file.tellg();
    std::cout << "✓ OpenSans-Bold.ttf found, size: " << file_size << " bytes\n";
    font_file.close();
    
    if (file_size > 50000 && file_size < 200000) { // Reasonable TTF size range
      std::cout << "✓ Font file size looks reasonable for TTF\n";
    } else {
      std::cout << "⚠ Font file size unusual: " << file_size << " bytes\n";
    }
  } else {
    std::cout << "✗ Cannot access OpenSans-Bold.ttf font file\n";
    std::cout << "  Expected location: ../assets/fonts/OpenSans-Bold.ttf\n";
  }
}

void TestFontLoading() {
  std::cout << "=== FontRenderer Loading Test ===\n";
  
  FontRenderer font_renderer;
  
  // Test initialization with OpenSans
  bool success = font_renderer.InitializeWithOpenSans(24.0f);
  
  if (success) {
    std::cout << "✓ Font loaded successfully\n";
    std::cout << "✓ Atlas texture ID: " << font_renderer.GetAtlasTexture() << "\n";
    std::cout << "✓ Atlas dimensions: " << font_renderer.GetAtlasWidth() 
              << "x" << font_renderer.GetAtlasHeight() << "\n";
    std::cout << "✓ Font initialized: " << (font_renderer.IsInitialized() ? "Yes" : "No") << "\n";
  } else {
    std::cout << "✗ Font loading failed\n";
    return;
  }
  
  // Test text metrics
  std::vector<std::string> test_texts = {
    "Hello",
    "World",
    "CheckPoint B",
    "X-Axis",
    "Y-Axis", 
    "Z-Axis"
  };
  
  std::cout << "\n=== Text Metrics Test ===\n";
  for (const auto& text : test_texts) {
    auto metrics = font_renderer.GetTextMetrics(text);
    std::cout << "Text: \"" << text << "\"\n";
    std::cout << "  Size: " << metrics.width << "x" << metrics.height << " pixels\n";
    std::cout << "  Ascent: " << metrics.ascent << ", Descent: " << metrics.descent << "\n";
    
    // Test vertex generation
    auto vertices = font_renderer.GenerateTextVertices(text, glm::vec3(0,0,0), 1.0f);
    std::cout << "  Vertices generated: " << vertices.size() << " (should be " 
              << (text.length() * 6) << " for " << text.length() << " chars)\n";
    std::cout << "\n";
  }
}

void TestGlyphInfo() {
  std::cout << "=== Glyph Info Test ===\n";
  
  FontRenderer font_renderer;
  if (!font_renderer.InitializeWithOpenSans(24.0f)) {
    std::cout << "✗ Font initialization failed\n";
    return;
  }
  
  // Test individual glyph info
  std::vector<char> test_chars = {'A', 'B', 'a', 'b', ' ', 'X', 'Y', 'Z'};
  
  for (char c : test_chars) {
    const auto* glyph = font_renderer.GetGlyph(c);
    if (glyph) {
      std::cout << "Glyph '" << c << "':\n";
      std::cout << "  Size: " << glyph->width << "x" << glyph->height << "\n";
      std::cout << "  Advance: " << glyph->advance_x << "\n";
      std::cout << "  Bearing: " << glyph->bearing_x << ", " << glyph->bearing_y << "\n";
      std::cout << "  Tex coords: (" << glyph->tex_x0 << "," << glyph->tex_y0 
                << ") to (" << glyph->tex_x1 << "," << glyph->tex_y1 << ")\n\n";
    } else {
      std::cout << "✗ Glyph '" << c << "' not found\n";
    }
  }
}

void TestTextVertexGeneration() {
  std::cout << "=== Text Vertex Generation Test ===\n";
  
  FontRenderer font_renderer;
  if (!font_renderer.InitializeWithOpenSans(16.0f)) {
    std::cout << "✗ Font initialization failed\n";
    return;
  }
  
  std::string test_text = "AB";
  auto vertices = font_renderer.GenerateTextVertices(test_text, glm::vec3(0,0,0), 1.0f);
  
  std::cout << "Text: \"" << test_text << "\"\n";
  std::cout << "Generated " << vertices.size() << " vertices\n";
  
  // Print first few vertices to verify positioning
  for (size_t i = 0; i < std::min(size_t(12), vertices.size()); i++) {
    const auto& v = vertices[i];
    std::cout << "  Vertex " << i << ": pos(" << v.position.x << "," << v.position.y << "," << v.position.z 
              << ") tex(" << v.tex_coord.x << "," << v.tex_coord.y << ")\n";
  }
}

void TestVisualRendering(GLFWwindow* window) {
  std::cout << "=== Visual Rendering Test ===\n";
  std::cout << "Opening window to display FontRenderer text...\n";
  
  // Create font renderer
  FontRenderer font_renderer;
  if (!font_renderer.InitializeWithOpenSans(48.0f)) {
    std::cout << "✗ Font initialization failed\n";
    return;
  }
  
  // Create simple shaders for text rendering
  const char* vertex_shader_source = R"(
    #version 330 core
    layout (location = 0) in vec3 aPosition;
    layout (location = 1) in vec2 aTexCoord;
    
    uniform mat4 uProjection;
    
    out vec2 vTexCoord;
    
    void main() {
      gl_Position = uProjection * vec4(aPosition, 1.0);
      vTexCoord = aTexCoord;
    }
  )";
  
  const char* fragment_shader_source = R"(
    #version 330 core
    in vec2 vTexCoord;
    
    out vec4 FragColor;
    
    uniform sampler2D uFontAtlas;
    uniform vec3 uTextColor;
    
    void main() {
      float alpha = texture(uFontAtlas, vTexCoord).r;
      if (alpha < 0.01) discard;
      FragColor = vec4(uTextColor, alpha);
    }
  )";
  
  // Compile shaders
  unsigned int vertex_shader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertex_shader, 1, &vertex_shader_source, nullptr);
  glCompileShader(vertex_shader);
  
  unsigned int fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragment_shader, 1, &fragment_shader_source, nullptr);
  glCompileShader(fragment_shader);
  
  unsigned int shader_program = glCreateProgram();
  glAttachShader(shader_program, vertex_shader);
  glAttachShader(shader_program, fragment_shader);
  glLinkProgram(shader_program);
  
  glDeleteShader(vertex_shader);
  glDeleteShader(fragment_shader);
  
  // Test texts to display
  std::vector<std::string> test_texts = {
    "FontRenderer Test",
    "Hello World!",
    "CheckPoint B",
    "X-Axis Y-Axis Z-Axis",
    "OpenSans Bold Font"
  };
  
  // Generate text geometry for all texts
  struct TextRender {
    std::string text;
    std::vector<FontRenderer::TextVertex> vertices;
    unsigned int vao, vbo;
    float y_pos;
  };
  
  std::vector<TextRender> text_renders;
  float y_start = 450.0f;  // Start higher on screen
  float y_spacing = 60.0f; // Spacing between lines
  
  for (size_t i = 0; i < test_texts.size(); ++i) {
    const auto& text = test_texts[i];
    TextRender tr;
    tr.text = text;
    tr.y_pos = y_start - (i * y_spacing);
    
    // Get text metrics to center horizontally
    auto metrics = font_renderer.GetTextMetrics(text);
    float x_center = 400.0f - (metrics.width * 0.5f);  // Center in 800px width
    
    // Generate vertices with centered position
    tr.vertices = font_renderer.GenerateTextVertices(text, glm::vec3(x_center, tr.y_pos, 0.0f), 1.0f);
    
    // Create VAO and VBO
    glGenVertexArrays(1, &tr.vao);
    glGenBuffers(1, &tr.vbo);
    
    glBindVertexArray(tr.vao);
    glBindBuffer(GL_ARRAY_BUFFER, tr.vbo);
    glBufferData(GL_ARRAY_BUFFER, tr.vertices.size() * sizeof(FontRenderer::TextVertex), 
                 tr.vertices.data(), GL_STATIC_DRAW);
    
    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(FontRenderer::TextVertex), (void*)0);
    glEnableVertexAttribArray(0);
    
    // Texture coordinate attribute  
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(FontRenderer::TextVertex), 
                          (void*)offsetof(FontRenderer::TextVertex, tex_coord));
    glEnableVertexAttribArray(1);
    
    text_renders.push_back(tr);
  }
  
  // Setup projection matrix (2D orthographic)
  glm::mat4 projection = glm::ortho(0.0f, 800.0f, 0.0f, 600.0f);
  
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
  
  std::cout << "Displaying text. Press ESC to close window...\n";
  
  // Render loop
  while (!glfwWindowShouldClose(window)) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
      glfwSetWindowShouldClose(window, true);
    }
    
    glClear(GL_COLOR_BUFFER_BIT);
    
    // Use shader program
    glUseProgram(shader_program);
    
    // Set uniforms
    unsigned int proj_loc = glGetUniformLocation(shader_program, "uProjection");
    glUniformMatrix4fv(proj_loc, 1, GL_FALSE, glm::value_ptr(projection));
    
    unsigned int atlas_loc = glGetUniformLocation(shader_program, "uFontAtlas");
    glUniform1i(atlas_loc, 0);
    
    unsigned int color_loc = glGetUniformLocation(shader_program, "uTextColor");
    glUniform3f(color_loc, 1.0f, 1.0f, 1.0f); // White text
    
    // Bind font atlas texture
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, font_renderer.GetAtlasTexture());
    
    // Render all text
    for (const auto& tr : text_renders) {
      glBindVertexArray(tr.vao);
      glDrawArrays(GL_TRIANGLES, 0, tr.vertices.size());
    }
    
    glfwSwapBuffers(window);
    glfwPollEvents();
  }
  
  // Cleanup
  for (auto& tr : text_renders) {
    glDeleteVertexArrays(1, &tr.vao);
    glDeleteBuffers(1, &tr.vbo);
  }
  glDeleteProgram(shader_program);
}

int main() {
  std::cout << "FontRenderer Verification Test\n";
  std::cout << "==============================\n\n";
  
  // Create minimal OpenGL context for testing
  MinimalOpenGLContext context;
  if (!context.Initialize()) {
    std::cout << "✗ Failed to create OpenGL context\n";
    return 1;
  }
  
  TestFontFileAccess();
  std::cout << "\n";
  
  TestFontLoading();
  std::cout << "\n";
  
  TestGlyphInfo();
  std::cout << "\n";
  
  TestTextVertexGeneration();
  std::cout << "\n";
  
  TestVisualRendering(context.GetWindow());
  
  std::cout << "\nFontRenderer test completed!\n";
  return 0;
}