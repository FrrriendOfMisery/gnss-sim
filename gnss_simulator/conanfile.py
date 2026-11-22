from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMakeDeps, CMake

class GnssSimConan(ConanFile):
    name = "gr-gnss-sim"
    version = "0.1.0"
    settings = "os", "compiler", "build_type", "arch"
    
    def requirements(self):
        # GNU Radio 4 пока нет в Conan Center, добавляем из git
        self.requires("gnuradio/4.0.0")
        self.requires("fmt/10.2.1")
    
    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.generate()
    
    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()