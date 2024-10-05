from conans import ConanFile, CMake


class QuickVizConan(ConanFile):
    name = "quickviz"
    version = "0.2.0"
    license = "MIT"
    author = "<Ruixiang Du> <ruixiang.du@gmail.com>"
    url = "<Package recipe repository url here, for issues about the package>"
    description = "https://github.com/rxdu/quickviz"
    topics = ("imgui", "implot", "cairo", "visualization", "gui")
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False]}
    default_options = {"shared": False}
    generators = "cmake_paths"
    exports_sources = "*"

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
        
    def package(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.install()

    def package_info(self):
        self.cpp_info.includedirs = ["include"]
        self.cpp_info.libs = ["imview"]

