from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout


class DepthCodecConan(ConanFile):
    name = "depth_codec"
    version = "0.1.0"
    description = "Lossless compression of 32FC1 depth images (dictionary + 2D MED prediction + ZSTD)"
    url = "https://github.com/facontidavide/depth_image_compression"
    settings = "os", "compiler", "build_type", "arch"
    exports_sources = "CMakeLists.txt", "include/*", "src/*", "test/*", "benchmark/*"

    def requirements(self):
        self.requires("zstd/[>=1.5 <1.6]")

    def layout(self):
        cmake_layout(self)

    def generate(self):
        tc = CMakeToolchain(self)
        # Packaged binaries must be portable; benchmarks are dev-only tools.
        tc.cache_variables["DEPTH_CODEC_MARCH_NATIVE"] = False
        tc.cache_variables["DEPTH_CODEC_BUILD_BENCHMARKS"] = False
        tc.generate()
        CMakeDeps(self).generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
        if not self.conf.get("tools.build:skip_test", default=False, check_type=bool):
            cmake.test()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = ["depthcodec"]
