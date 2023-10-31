from ..package import SingularPackageFile, package_file


@package_file
class PackageXML(SingularPackageFile):
    @classmethod
    def name(cls):
        return 'package.xml'
