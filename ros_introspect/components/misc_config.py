from ..package import PackageFile, package_file

MISC_CONFIGS = {
    '.travis.yml',
    '.pre-commit-config.yaml',
}


@package_file
class MiscConfig(PackageFile):
    @classmethod
    def is_type(cls, path):
        if path.name in MISC_CONFIGS:
            return True
        if '.github' in path.parts:
            return True
        return False
