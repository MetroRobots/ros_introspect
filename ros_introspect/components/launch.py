from ..package import PackageFile, package_file, DependencyType
import re
from xml.dom.minidom import parseString as parse_xml
from xml.parsers.expat import ExpatError


class Launch(PackageFile):
    def get_dependencies(self, dependency_type):
        deps = set()
        if dependency_type != DependencyType.RUN:
            return deps

        deps.update(self.get_node_pkgs())
        deps.update(self.get_include_pkgs())
        deps.update(self.get_misc_pkgs())
        return deps


@package_file
class LaunchXML(Launch):
    def __init__(self, full_path, package_root):
        super().__init__(full_path, package_root)
        try:
            self.tree = parse_xml(full_path)
            self.is_test = len(self.tree.getElementsByTagName('test')) > 0
        except ExpatError:  # this is an invalid xml file
            self.is_test = False

    @classmethod
    def is_type(cls, path):
        if path.suffix == '.launch':
            return True
        if path.suffix != '.xml':
            return False
        with open(path) as f:
            line = f.readline()
            return '<launch' in line

    def get_node_pkgs(self):
        s = set()
        for node in self.tree.getElementsByTagName('node'):
            s.add(str(node.getAttribute('pkg')))
        return sorted(s)

    def get_include_pkgs(self):
        s = set()
        for node in self.tree.getElementsByTagName('include'):
            el = node.getAttribute('file')
            if 'find' in el:
                i = el.index('find')
                i2 = el.index(')', i)
                s.add(el[i + 5:i2])
        return sorted(s)

    def get_misc_pkgs(self):
        s = set()
        xml_str = self.tree.toxml()
        for x in re.finditer(r'\$\(find ([^\)]*)\)', xml_str):
            s.add(x.group(1))
        # rosrun PKG (e.g. <param command="rosrun xacro xacro.py xacrofile.xacro" />
        for x in re.finditer(r'rosrun\s+(\w+)\s', xml_str):
            s.add(x.group(1))
        return s


PY_NODE_PATTERN = re.compile(r'package=["\']([\w_]+)["\']')
PY_SHARE_PATTERN = re.compile(r'get_package_share_(path|directory)\(\s*["\']([\w_]+)["\']')


class LaunchPy(Launch):
    def __init__(self, rel_fn, file_path):
        Launch.__init__(self, rel_fn, file_path)
        self.test = False
        self.contents = open(file_path).read()

    @classmethod
    def is_type(cls, path):
        path_s = str(path)
        if path_s.endswith('.launch.py'):
            return True

    def get_node_pkgs(self):
        s = set()
        for m in PY_NODE_PATTERN.finditer(self.contents):
            pkg = m.group(1)
            s.add(pkg)
        return sorted(s)

    def get_include_pkgs(self):
        s = set()
        for m in PY_SHARE_PATTERN.finditer(self.contents):
            pkg = m.group(2)
            s.add(pkg)
        return sorted(s)

    def get_misc_pkgs(self):
        return set()
