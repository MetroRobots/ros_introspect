from ..package import PackageFile, package_file

MISC_CONFIGS = {
    'bitbucket-pipelines.yml',
    '.clang-format'
}


@package_file
class MiscConfig(PackageFile):
    @classmethod
    def is_type(cls, path):
        if path.name in MISC_CONFIGS:
            return True
        elif path.name and path.name[0] == '.' and path.suffix in ['.yaml', '.yml']:
            return True
        elif '.github' in path.parts:
            return True
        return False
