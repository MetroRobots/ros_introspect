from ..package import PackageFile, package_file
from stylish_cmake_parser import parse_file

ROS_TESTING_FLAGS = ['CATKIN_ENABLE_TESTING', 'BUILD_TESTING']


def is_testing_group(command_group):
    cmd = command_group.initial_cmd
    return cmd.command_name == 'if' and cmd.first_token() in ROS_TESTING_FLAGS


@package_file
class CMake(PackageFile):
    def __init__(self, full_path, package):
        super().__init__(full_path, package)
        self.contents = parse_file(self.full_path)
        self.is_metapackage = len(self.contents.content_map['catkin_metapackage']) > 0

    @classmethod
    def is_type(cls, path):
        return path.name == 'CMakeLists.txt'

    @classmethod
    def attribute_name(cls):
        return 'cmakes'

    def get_libraries(self):
        return list(self.contents.get_build_rules('add_library').keys())

    def get_executables(self):
        return list(self.contents.get_build_rules('add_executable').keys())

    def get_test_sections(self):
        for command_group in self.contents.content_map['group']:
            if is_testing_group(command_group):
                yield command_group.contents

    def get_test_source(self):
        test_files = set()
        for section in self.get_test_sections():
            test_files.update(section.get_library_source())
            test_files.update(section.get_executable_source())
        return test_files

    def get_source_tags(self):
        return {
            'library': self.contents.get_library_source(),
            'executable': self.contents.get_executable_source(),
            'test': self.get_test_source(),
        }

    def write(self, output_path):
        with open(output_path, 'w') as f:
            f.write(str(self.contents))
