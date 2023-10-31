from ..package import SingularPackageFile, package_file
from xml.dom.minidom import parseString as parse_xml
import re
import collections
import operator

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
        self.root = self.tree.getElementsByTagName('package')[0]

        self.header = contents[:get_package_tag_index(contents)]
        self._name = None
        self._format = None
        self._std_tab = None
        self._build_type = None

    @classmethod
    def category_name(cls):
        return 'package.xml'

    @property
    def name(self):
        if self._name is not None:
            return self._name
        name_tags = self.root.getElementsByTagName('name')
        if not name_tags:
            return
        name_tag = name_tags[0]
        self._name = name_tag.firstChild.nodeValue
        return self._name

    @property
    def xml_format(self):
        if self._format is not None:
            return self._format
        if not self.root.hasAttribute('format'):
            self._format = 1
        else:
            self._format = int(self.root.attributes['format'].value)
        return self._format

    @property
    def build_type(self):
        if self._build_type is not None:
            return self._build_type

        build_types = set()

        for tag in self.root.getElementsByTagName('build_type') + self.root.getElementsByTagName('buildtool_depend'):
            value = tag.firstChild.nodeValue
            if value in BUILD_TYPES:
                build_types.add(value)

        if len(build_types) == 1:
            self._build_type = list(build_types)[0]
            return self._build_type
        elif not build_types:
            raise RuntimeError('Unable to determine buildtool type in {}'.format(self.full_path))
        else:
            raise RuntimeError('Too many valid buildtool types')

    @property
    def std_tab(self):
        if self._std_tab is not None:
            return self._std_tab
        tab_ct = collections.defaultdict(int)
        for c in self.root.childNodes:
            if c.nodeType == c.TEXT_NODE:
                spaces = count_trailing_spaces(c.data)
                tab_ct[spaces] += 1
        if len(tab_ct) == 0:
            self._std_tab = 4
        else:
            self._std_tab = max(tab_ct.items(), key=operator.itemgetter(1))[0]
        return self._std_tab

    def save(self, output_path):
        s = self.tree.toxml(self.tree.encoding)
        index = get_package_tag_index(s)
        s = self.header + s[index:] + '\n'

        with open(output_path, 'wb') as f:
            f.write(s.encode('UTF-8'))
