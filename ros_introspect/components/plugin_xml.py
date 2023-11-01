from ..package import PackageFile, package_file
from collections import OrderedDict
from xml.dom.minidom import parse as parse_xml_file
from xml.parsers.expat import ExpatError

NS_PATTERN = '%s::%s'


@package_file
class PluginXML(PackageFile):

    def __init__(self, full_path, package):
        super().__init__(full_path, package)

        self.library_prefix = (package.ros_version == 1)
        self.has_class_libraries_tag = False
        self.libraries = OrderedDict()
        self.parent_pkgs = set()

        if self.full_path.exists():
            self.read()

    def read(self):
        try:
            tree = parse_xml_file(open(self.full_path))
        except ExpatError:
            return

        self.has_class_libraries_tag = len(tree.getElementsByTagName('class_libraries')) > 0

        for el in tree.getElementsByTagName('library'):
            path = el.getAttribute('path')
            if self.library_prefix:
                path = path.replace('lib/lib', '')
            cls = OrderedDict()
            self.libraries[path] = cls

            for clstag in el.getElementsByTagName('class'):
                d = {}
                d['base_class_type'] = clstag.getAttribute('base_class_type')
                self.parent_pkgs.add(d['base_class_type'].split('::')[0])
                d['type'] = clstag.getAttribute('type')
                d['name'] = clstag.getAttribute('name')

                desc = ''
                for tag in clstag.getElementsByTagName('description'):
                    if len(tag.childNodes) == 0:
                        continue
                    desc += str(tag.childNodes[0].nodeValue)
                d['description'] = desc

                cls[d['type']] = d

    @classmethod
    def is_type(cls, path):
        if path.suffix != '.xml':
            return False
        with open(path) as f:
            line = f.readline()
            return '<library' in line or '<class_libraries' in line

    def contains_library(self, library_name, pkg, name):
        if library_name not in self.libraries and self.library_prefix:
            library_name = f'lib{library_name}'
        if library_name not in self.libraries:
            return False
        full_name = NS_PATTERN % (pkg, name)
        return full_name in self.libraries[library_name]

    def insert_new_class(self, library_name, pkg, name, base_pkg, base_name, description=''):
        if library_name not in self.libraries:
            self.libraries[library_name] = OrderedDict()
        library = self.libraries[library_name]
        full_name = NS_PATTERN % (pkg, name)
        library[full_name] = {'base_class_type': NS_PATTERN % (base_pkg, base_name),
                              'type': full_name,
                              'description': description}
        self.changed = True

    def write(self, output_path):
        with open(output_path, 'w') as f:
            f.write(str(self))

    def __repr__(self):
        s = ''
        indent = 0
        need_class_libraries_tag = len(self.libraries) > 1 or self.has_class_libraries_tag
        if need_class_libraries_tag:
            s += '<class_libraries>\n'
            indent += 2

        for name, lib in self.libraries.items():
            library_name = 'lib/lib' + name if self.library_prefix else name
            s += ' ' * indent + '<library path="%s">\n' % library_name
            for clib in lib.values():
                s += self.class_str(clib, indent + 2)
            s += ' ' * indent + '</library>\n'

        if need_class_libraries_tag:
            s += '</class_libraries>\n'
        return s

    def class_str(self, lib, indent):
        s = ' ' * indent + '<class'
        for at in ['name', 'type', 'base_class_type']:
            if at in lib and len(lib[at]) > 0:
                s += ' %s="%s"' % (at, lib[at])
        s += '>\n' + (' ' * (indent + 2))
        s += '<description>%s</description>' % lib.get('description', '')

        s += '\n' + (' ' * indent) + '</class>\n'
        return s
