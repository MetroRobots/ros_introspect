import collections

from .cmake_parser import parse_file
from .launch import LaunchPy, LaunchXML
from .package_structure import get_package_structure
from .package_xml import PackageXML
from .plugin_xml import PluginXML
from .ros_generator import ROSGenerator
from .rviz_config import RVizConfig
from .setup_cfg import SetupCFG
from .setup_py import SetupPy
from .source_code import SourceCode
from .urdf import UrdfFile


class Package:
    def __init__(self, root):
        self.manifest = PackageXML(self.root + '/package.xml')
        self.name = self.manifest.name
        self.build_type = self.manifest.build_type
        self._ros_version = None
        self.cmake = parse_file(self.root + '/CMakeLists.txt')

        package_structure = get_package_structure(root)
        self.source_code = SourceCode(package_structure['source'], self.name)
        if self.cmake:
            self.source_code.setup_tags(self.cmake)

        self.launches = []
        self.plugin_configs = []
        self.urdf_files = []
        for rel_fn, file_path in package_structure['launchpy'].items():
            self.launches.append(LaunchPy(rel_fn, file_path))
        for rel_fn, file_path in package_structure['launch'].items():
            self.launches.append(LaunchXML(rel_fn, file_path))
        for rel_fn, file_path in package_structure['plugin_config'].items():
            self.plugin_configs.append(PluginXML(rel_fn, file_path, self.ros_version == 1))
        for rel_fn, file_path in package_structure['urdf'].items():
            self.urdf_files.append(UrdfFile(rel_fn, file_path))

        self.setup_py = None
        if 'setup.py' in package_structure['key']:
            self.setup_py = SetupPy(self.name, package_structure['key']['setup.py'], self.ros_version == 2)
        self.generators = collections.defaultdict(list)
        for rel_fn, path in package_structure['generators'].items():
            gen = ROSGenerator(rel_fn, path)
            self.generators[gen.type].append(gen)
        self.dynamic_reconfigs = package_structure['cfg'].keys()
        self.urdf_files = []
        for rel_fn, path in package_structure['urdf'].items():
            self.urdf_files.append(UrdfFile(rel_fn, path))
        self.rviz_configs = []
        self.setup_cfg = None
        for rel_fn, path in package_structure[None].items():
            if path.endswith('.rviz'):
                self.rviz_configs.append(RVizConfig(rel_fn, path))
            if rel_fn == 'setup.cfg':
                self.setup_cfg = SetupCFG(path)
        self.misc_files = list(package_structure[None].keys()) + list(package_structure['models'].keys())

    def is_metapackage(self):
        return self.manifest.is_metapackage() or (self.cmake and self.cmake.is_metapackage())

    def get_build_dependencies(self):
        return self.source_code.get_build_dependencies()

    def get_run_dependencies(self):
        packages = self.source_code.get_build_dependencies()
        for launch in self.launches:
            if launch.test:
                continue
            packages.update(launch.get_dependencies())

        for urdf in self.urdf_files:
            packages.update(urdf.get_dependencies())

        for rviz_config in self.rviz_configs:
            packages.update(rviz_config.get_dependencies())

        if self.name in packages:
            packages.remove(self.name)
        return packages

    def get_test_dependencies(self):
        packages = set()
        packages.update(self.source_code.get_test_dependencies())
        for launch in self.launches:
            if not launch.test:
                continue
            packages.add('rostest')
            packages.update(launch.get_dependencies())
        if self.name in packages:
            packages.remove(self.name)
        return packages

    def get_all_generators(self):
        for gens in self.generators.values():
            for gen in gens:
                yield gen

    def get_dependencies_from_msgs(self):
        packages = set()
        for gen in self.get_all_generators():
            packages.update(gen.dependencies)
        if self.name in packages:
            packages.remove(self.name)
        return packages

    @property
    def ros_version(self):
        if self._ros_version is None:
            if self.build_type == 'catkin':
                self._ros_version = 1
            else:
                self._ros_version = 2
        return self._ros_version
