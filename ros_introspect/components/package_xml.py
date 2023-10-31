from ..package import SingularPackageFile, package_file


@package_file
class PackageXML(SingularPackageFile):
    @classmethod
    def category_name(cls):
        return 'package.xml'
