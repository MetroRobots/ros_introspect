from ..package import PackageFile, package_file
from stylish_cmake_parser import parse_file


@package_file
class CMake(PackageFile):
    def __init__(self, full_path, package):
        super().__init__(full_path, package)
        self.contents = parse_file(self.full_path)

    @classmethod
    def is_type(cls, path):
        return path.name == 'CMakeLists.txt'

    def write(self, output_path):
        with open(output_path, 'w') as f:
            f.write(str(self.contents))
