from ..package import PackageFile, package_file, DependencyType
import re

FIND_PATTERN = re.compile(r'\$\(find ([^\)]+)\)')
PACKAGE_PATTERN = re.compile(r'package://([^/]+)/')


@package_file
class UrdfFile(PackageFile):
    def __init__(self, full_path, package):
        super().__init__(full_path, package)
        self.contents = open(full_path).read()

    @classmethod
    def is_type(cls, path):
        return path.suffix in ['.urdf', '.xacro']

    @classmethod
    def needs_share_installation(cls):
        return True

    def get_dependencies(self, dependency_type):
        deps = set()
        if dependency_type != DependencyType.RUN:
            return deps
        for pattern in [FIND_PATTERN, PACKAGE_PATTERN]:
            for match in pattern.findall(self.contents):
                deps.add(match)
        return deps
