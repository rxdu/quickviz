from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.scm import Git


class QuickVizConan(ConanFile):
    name = "quickviz"
    version = "0.6.5"
    license = "MIT"
    author = "<Ruixiang Du> <ruixiang.du@gmail.com>"
    url = "https://github.com/rxdu/quickviz"
    description = "C++ libraries for creating data visualization and basic UI applications in robotics"
    topics = ("imgui", "implot", "cairo", "visualization", "gui")
    settings = "os", "compiler", "build_type", "arch"
    default_settings = {"build_type": "Release"}
    options = {"shared": [True, False]}
    default_options = {"shared": False}

    def export(self):
        git = Git(self, self.recipe_folder)
        git.coordinates_to_conandata()

    def source(self):
        git = Git(self)
        git.checkout_from_conandata_coordinates()
        git.run("submodule update --init --recursive")
    
    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def layout(self):
        cmake_layout(self)

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        
        tc = CMakeToolchain(self)
        tc.variables["BUILD_TESTING"] = "OFF"
        if self.options.shared:
            tc.variables["CMAKE_POSITION_INDEPENDENT_CODE"] = "ON"
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.builddirs = ["lib/cmake"]
        self.cpp_info.set_property("cmake_find_mode", "none")

