from .cmake_parser import parse_file
from .package_structure import get_package_structure
from .plugin_xml import PluginXML
from .rviz_config import RVizConfig
from .source_code import SourceCode
from .urdf import UrdfFile


class Package:
    def __init__(self, root):
        self.cmake = parse_file(self.root + '/CMakeLists.txt')

        package_structure = get_package_structure(root)
        self.source_code = SourceCode(package_structure['source'], self.name)
        if self.cmake:
            self.source_code.setup_tags(self.cmake)

        self.plugin_configs = []
        self.urdf_files = []
        for rel_fn, file_path in package_structure['plugin_config'].items():
            self.plugin_configs.append(PluginXML(rel_fn, file_path, self.ros_version == 1))
        for rel_fn, file_path in package_structure['urdf'].items():
            self.urdf_files.append(UrdfFile(rel_fn, file_path))

        self.dynamic_reconfigs = package_structure['cfg'].keys()
        self.urdf_files = []
        for rel_fn, path in package_structure['urdf'].items():
            self.urdf_files.append(UrdfFile(rel_fn, path))
        self.rviz_configs = []

        for rel_fn, path in package_structure[None].items():
            if path.endswith('.rviz'):
                self.rviz_configs.append(RVizConfig(rel_fn, path))
        self.misc_files = list(package_structure[None].keys()) + list(package_structure['models'].keys())

    def is_metapackage(self):
        return self.manifest.is_metapackage() or (self.cmake and self.cmake.is_metapackage())

    def get_test_dependencies(self):
        packages = set()
        packages.update(self.source_code.get_test_dependencies())
        for launch in self.launches:
            if not launch.test:
                continue
            packages.add('rostest')
            packages.update(launch.get_dependencies())
        return packages
