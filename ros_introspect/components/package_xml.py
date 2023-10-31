from ..package import SingularPackageFile, package_file
from xml.dom.minidom import parseString as parse_xml
import re
import collections
import operator

Person = collections.namedtuple('Person', 'name email')

INITIAL_TAGS = ['name', 'version', 'description']
MBLOCK_TAGS = ['maintainer', 'license', 'url', 'author']
DEPEND_TAGS = ['buildtool_depend', 'buildtool_export_depend', 'depend', 'build_depend', 'build_export_depend',
               'run_depend', 'exec_depend', 'test_depend', 'doc_depend']
FINAL_TAGS = ['group_depend', 'member_of_group', 'export']

INDENT_PATTERN = re.compile('\n *')
PEOPLE_TAGS = ['maintainer', 'author']

BUILD_TYPES = {'catkin', 'ament_python', 'ament_cmake', 'cmake'}

FORMAT_3_HEADER = """<?xml version="1.0"?>
<?xml-model
  href="http://download.ros.org/schema/package_format3.xsd"
  schematypens="http://www.w3.org/2001/XMLSchema"?>
"""


def get_package_tag_index(s, key='<package'):
    if key not in s:
        return 0
    return s.index(key)


def count_trailing_spaces(s):
    c = 0
    while c < len(s):
        if s[-c - 1] == ' ':
            c += 1
        elif s[-c - 1] == '\t':
            c += 4
        else:
            break
    return c


@package_file
class PackageXML(SingularPackageFile):

    def __init__(self, full_path, package_root):
        super().__init__(full_path, package_root)
        contents = open(full_path).read()
        self.tree = parse_xml(contents)
        package_tags = self.tree.getElementsByTagName('package')
        if not package_tags:
            self.tree = parse_xml('<package />')
            package_tags = self.tree.getElementsByTagName('package')
        self.root = package_tags[0]

        self.header = contents[:get_package_tag_index(contents)]

    @classmethod
    def category_name(cls):
        return 'package.xml'

    @property
    def name(self):
        name_tags = self.root.getElementsByTagName('name')
        if not name_tags:
            return
        name_tag = name_tags[0]
        return name_tag.firstChild.nodeValue

    @property
    def xml_format(self):
        if not self.root.hasAttribute('format'):
            return 1
        else:
            return int(self.root.attributes['format'].value)

    @property
    def build_type(self):
        build_types = set()

        for tag in self.root.getElementsByTagName('build_type') + self.root.getElementsByTagName('buildtool_depend'):
            value = tag.firstChild.nodeValue
            if value in BUILD_TYPES:
                build_types.add(value)

        if len(build_types) == 1:
            return list(build_types)[0]
        elif not build_types:
            raise RuntimeError('Unable to determine buildtool type in {}'.format(self.full_path))
        else:
            raise RuntimeError('Too many valid buildtool types')

    @property
    def std_tab(self):
        tab_ct = collections.defaultdict(int)
        for c in self.root.childNodes:
            if c.nodeType == c.TEXT_NODE:
                spaces = count_trailing_spaces(c.data)
                tab_ct[spaces] += 1
        if tab_ct:
            return max(tab_ct.items(), key=operator.itemgetter(1))[0]

    def get_elements_by_tags(self, tags):
        elements = []
        for tag in tags:
            elements += self.root.getElementsByTagName(tag)
        return elements

    def get_packages_by_tag(self, tag):
        pkgs = []
        for el in self.root.getElementsByTagName(tag):
            pkgs.append(el.childNodes[0].nodeValue)
        return pkgs

    def get_people(self):
        people = []
        for el in self.get_elements_by_tags(PEOPLE_TAGS):
            name = el.childNodes[0].nodeValue
            email = el.getAttribute('email')
            people.append(Person(name, email))
        return people

    def get_license_element(self):
        els = self.root.getElementsByTagName('license')
        if els:
            return els[0]

    def get_license(self):
        el = self.get_license_element()
        if el:
            return el.childNodes[0].nodeValue

    def is_metapackage(self):
        for node in self.root.getElementsByTagName('export'):
            for child in node.childNodes:
                if child.nodeType == child.ELEMENT_NODE:
                    if child.nodeName == 'metapackage':
                        return True
        return False

    def write(self, output_path):
        s = self.tree.toxml(self.tree.encoding)
        index = get_package_tag_index(s)
        s = self.header + s[index:] + '\n'

        with open(output_path, 'wb') as f:
            f.write(s.encode('UTF-8'))
