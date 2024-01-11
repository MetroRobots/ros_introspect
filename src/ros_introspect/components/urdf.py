from ..package import PackageTextFile, package_file, DependencyType
import re

FIND_PATTERN = re.compile(r'\$\(find ([^\)]+)\)')
PACKAGE_PATTERN = re.compile(r'package://([^/]+)/')


@package_file
class UrdfFile(PackageTextFile):
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
