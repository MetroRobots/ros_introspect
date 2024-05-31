from ..package import PackageFile, package_file

MISC_CONFIGS = {
    'bitbucket-pipelines.yml',
}


@package_file
class MiscConfig(PackageFile):
    @classmethod
    def is_type(cls, path):
        if path.name in MISC_CONFIGS:
            return True
        return False
