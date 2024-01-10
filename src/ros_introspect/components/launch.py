from ..package import PackageTextFile, package_file, DependencyType
import re
from xml.dom.minidom import parseString as parse_xml
from xml.parsers.expat import ExpatError


class Launch(PackageTextFile):
    def get_dependencies(self, dependency_type):
        deps = set()
        if self.is_test:
            # TODO: Maybe add rostest
            if dependency_type != DependencyType.TEST:
                return deps
        else:
            if dependency_type != DependencyType.RUN:
                return deps

        deps.update(self.get_node_pkgs())
        deps.update(self.get_include_pkgs())
        deps.update(self.get_misc_pkgs())
        return deps

    @classmethod
    def needs_share_installation(cls):
        return True


@package_file
class LaunchXML(Launch):
    def __init__(self, full_path, package):
        super().__init__(full_path, package)
        try:
            self.tree = parse_xml(self.contents)
            self.is_test = len(self.tree.getElementsByTagName('test')) > 0
        except (ExpatError, FileNotFoundError, TypeError):  # pragma: no cover (invalid/nonexistent launch xml)
            self.tree = parse_xml('<launch />')
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

    def regenerate_contents(self):
        return self.tree.toxml(self.tree.encoding)


PY_NODE_PATTERN = re.compile(r'package=["\']([\w_]+)["\']')
PY_SHARE_PATTERN = re.compile(r'get_package_share_(path|directory)\(\s*["\']([\w_]+)["\']')
PY_FIND_PATTERN = re.compile(r'FindPackageShare\(\'(\w+)\'\)')


@package_file
class LaunchPy(Launch):
    def __init__(self, full_path, package):
        super().__init__(full_path, package)
        self.is_test = self.rel_fn.parts[0] == 'test'

    @classmethod
    def is_type(cls, path):
        path_s = str(path)
        return path_s.endswith('.launch.py')

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
        s = set()
        for m in PY_FIND_PATTERN.finditer(self.contents):
            pkg = m.group(1)
            s.add(pkg)
        if 'xacro' in self.contents:
            s.add('xacro')
        return sorted(s)
