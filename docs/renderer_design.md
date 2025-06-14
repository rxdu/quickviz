# Renderer Design

For data visualization, the renderer library provides a set of API functions to easily plot 2D time-series data, draw 2D primitives and render 3D objects. It can be used to visualize data in real-time.

## Implementation

The rendering pipeline uses the render-to-texture approach and is built on top of the core components described above.

```mermaid
---
title: rendering pipeline
---
classDiagram
    SceneObject <|-- Panel
    Panel <|-- GlSceneManager
    GlSceneManager o--	OpenGlObject
    GlSceneManager o-- FrameBuffer
    GlSceneManager o-- CameraController
    Camera <-- CameraController
    OpenGlObject <|-- Grid
    ShaderProgram *-- PointCloud
    Shader --> ShaderProgram
    OpenGlObject <|-- Triangle
    OpenGlObject <|-- PointCloud


    namespace quickviz {
        class SceneObject {
            <<Interface>>
            +OnRender() void *
        }
        class Panel {
            +SetAutoLayout(bool value) void
            #Begin() void
            #End() void
        }     
        class GlSceneManager {
            +AddOpenGlObject(const std::string& name, std::unique_ptr<OpenGlObject> object) void
        }
        class OpenGlObject {
            <<Interface>>
            +OnDraw(const glm::mat4& projection, const glm::mat4& view) void
        }
        class FrameBuffer {
            +Bind() void
            +Unbind() void
            +Clear() void
            +GetTextureId() uint32_t
        }
        class Camera {
            +SetPosition(const glm::vec3& position) void
            +SetRotation(const glm::vec3& rotation) void
            +SetFOV(float fov) void
            +SetNearPlane(float near) void
            +SetFarPlane(float far) void
        }
        class CameraController {
            +SetCamera(std::shared_ptr<Camera> camera) void
            +Update(float dt) void
        }   
        class Shader {
            +Compile() void
        }
        class ShaderProgram {
            +AttachShader(const Shader& shader) void
            +LinkProgram() bool
            +Use() void
            +SetUniform(const std::string& name, const glm::vec3& value) void
            +SetUniform(const std::string& name, const glm::mat4& value) void
        }
        class Grid {
            +SetLineColor(const glm::vec3& color, float alpha) void
        }
        class Triangle {
            +SetColor(const glm::vec3& color, float alpha) void
        }   
        class PointCloud {
            +SetPointSize(float size) void
            +SetOpacity(float opacity) void
            +SetRenderMode(PointRenderMode mode) void
        }
    }
```
