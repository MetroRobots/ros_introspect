from .package_structure import get_package_structure


class Package:
    def __init__(self, root):
        package_structure = get_package_structure(root)
        if self.cmake:
            self.source_code.setup_tags(self.cmake)

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
