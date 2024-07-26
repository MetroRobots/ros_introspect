from ..package import SingularPackageFile, PackageTextFile, package_file, DependencyType
from xml.dom.minidom import parseString as parse_xml
from xml.parsers.expat import ExpatError
import re
import sys
import collections
from colorama import Fore
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

# The depend tag is equivalent to these three tags
DEPEND_TYPES = [DependencyType.BUILD, DependencyType.BUILD_EXPORT, DependencyType.RUN]
DEPENDENCY_TAG_MATCHING_V1 = {
    DependencyType.BUILD: 'build_depend',
    DependencyType.RUN: 'run_depend',
    DependencyType.TEST: 'test_depend',
}
DEPENDENCY_TAG_MATCHING = {
    DependencyType.BUILD: 'build_depend',
    DependencyType.RUN: 'exec_depend',
    DependencyType.TEST: 'test_depend',
    DependencyType.BUILD_EXPORT: 'build_export_depend',
}


# In most manifests, the ordering of the mblock doesn't matter, but we sort the depends
ORDERING = INITIAL_TAGS + [MBLOCK_TAGS] + DEPEND_TAGS + FINAL_TAGS
# In V3 manifests, we ensure the mblock is sorted, but not the depends
ORDERING_V3 = INITIAL_TAGS + MBLOCK_TAGS + DEPEND_TAGS + FINAL_TAGS


def get_ordering_index(name, whiny=True, manifest_version=None):
    if manifest_version and manifest_version >= 3:
        ordering = ORDERING_V3
    else:
        ordering = ORDERING

    for i, o in enumerate(ordering):
        if isinstance(o, list):
            if name in o:
                return i
        elif name == o:
            return i
    if name and whiny:
        print(f'\t{Fore.YELLOW}Unsure of ordering for {name}{Fore.RESET}', file=sys.stderr)  # pragma: no cover
    return len(ordering)


def get_sort_key(node, alphabetize_depends=True, manifest_version=None):
    if node:
        name = node.nodeName
    else:
        name = None

    index = get_ordering_index(name, manifest_version=manifest_version)

    if not alphabetize_depends:
        return index
    if name and 'depend' in name:
        return index, node.firstChild.data
    else:
        return index, ''


def get_chunks(children):
    """Given the children, group the elements into tuples.

    Tuple format: (an element node, [(some number of text nodes), that element node again])
    """
    chunks = []
    current = []
    for child_node in children:
        current.append(child_node)
        if child_node.nodeType == child_node.ELEMENT_NODE:
            chunks.append((child_node, current))
            current = []
    if len(current) > 0:
        chunks.append((None, current))
    return chunks


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
class PackageXML(SingularPackageFile, PackageTextFile):

    def __init__(self, full_path, package):
        PackageTextFile.__init__(self, full_path, package)
        try:
            self.tree = parse_xml(self.contents)
            self.root = self.tree.getElementsByTagName('package')[0]
            self.header = self.contents[:get_package_tag_index(self.contents)]
        except (ExpatError, IndexError, TypeError):
            self.tree = parse_xml('<package />')
            package_tags = self.tree.getElementsByTagName('package')
            self.root = package_tags[0]
            self.header = ''

    @classmethod
    def category_name(cls):
        return 'package.xml'

    @classmethod
    def attribute_name(cls):
        return 'package_xml'

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

    def create_new_tab_element(self, tabs=1):
        return self.tree.createTextNode('\n' + ' ' * (self.std_tab * tabs))

    def get_elements_by_tags(self, tags):
        elements = []
        for tag in tags:
            elements += self.root.getElementsByTagName(tag)
        return elements

    def get_packages_by_tag(self, tag):
        pkgs = set()
        for el in self.root.getElementsByTagName(tag):
            pkgs.add(el.childNodes[0].nodeValue)
        return pkgs

    # The package xml has no implicit dependencies.
    # This is just to look up what's already in the package
    def lookup_dependencies(self, dependency_type):
        keys = []
        if self.xml_format == 1:
            if dependency_type in DEPENDENCY_TAG_MATCHING_V1:
                keys.append(DEPENDENCY_TAG_MATCHING_V1[dependency_type])
        else:
            keys.append(DEPENDENCY_TAG_MATCHING[dependency_type])
            if dependency_type in DEPEND_TYPES:
                keys.append('depend')

        pkgs = set()
        for key in keys:
            pkgs |= self.get_packages_by_tag(key)
        return pkgs

    def get_people_by_tag(self, tag):
        people = []
        for el in self.root.getElementsByTagName(tag):
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

    def set_license(self, license_str):
        el = self.get_license_element()
        if not el:
            # TODO: Create license element
            pass

        if license != el.childNodes[0].nodeValue:
            el.childNodes[0].nodeValue = license_str
            self.changed = True

    def contains_node(self, node_name):
        return bool(self.root.getElementsByTagName(node_name))

    def get_insertion_index(self, tag):
        """Return the index where to insert a new element"""
        new_key = get_sort_key(tag, manifest_version=self.xml_format)
        prev_i = None
        for i, child in enumerate(self.root.childNodes):
            if child.nodeType == child.TEXT_NODE:
                continue
            key = get_sort_key(child, manifest_version=self.xml_format)

            if key < new_key:
                prev_i = i
            elif new_key < key:
                return prev_i + 1
        return len(self.root.childNodes) - 2

    def create_new_tag(self, node_name, node_text=None):
        node = self.tree.createElement(node_name)
        if node_text:
            node.appendChild(self.tree.createTextNode(node_text))
        self.insert_new_tag(node)
        return node

    def insert_new_tag(self, tag, index=None):
        if index is None:
            index = self.get_insertion_index(tag)

        before = self.root.childNodes[:index + 1]
        after = self.root.childNodes[index + 1:]

        new_tab_element = self.create_new_tab_element()

        # if the tag immediately before where we're going to insert is a text node,
        # then insert the new element and then the tab
        if before and before[-1].nodeType == before[-1].TEXT_NODE:
            new_bits = [tag, new_tab_element]
        else:
            # Otherwise (i.e. most cases) insert the tab then the element
            new_bits = [new_tab_element, tag]

        self.root.childNodes = before + new_bits + after
        self.changed = True

    def insert_new_packages(self, tag, values):
        for pkg in sorted(values):
            self.create_new_tag(tag, pkg)

    def add_dependencies(self, dependency_dict, prefer_depend_tag=False):
        if self.xml_format == 1:
            dependency_dict[DependencyType.RUN].update(dependency_dict[DependencyType.BUILD])

            # Special handling because message_generation is never a run dep
            dependency_dict[DependencyType.RUN].discard('message_generation')

        # Get Existing Dependencies
        existing_deps = {}
        for dep_type in DependencyType:
            existing_deps[dep_type] = self.lookup_dependencies(dep_type)

        if self.xml_format == 1:
            for dep_type in [DependencyType.BUILD, DependencyType.RUN]:
                self.insert_new_packages(DEPENDENCY_TAG_MATCHING_V1[dep_type],
                                         dependency_dict[dep_type] - existing_deps[dep_type])
        elif prefer_depend_tag:
            # If prefer depend tag is True, any time you have a BUILD, EXPORT or RUN dependency, just add depend
            depend_tags = set()
            for dep_type in DEPEND_TYPES:
                depend_tags |= dependency_dict[dep_type]
            depend_tags.discard('message_generation')
            depend_tags.discard('message_runtime')
            depend_tags.discard('rosidl_default_generators')
            depend_tags.discard('rosidl_default_runtime')

            # Remove tags that overlap with new depends
            for dep_type in DEPEND_TYPES:
                tag_name = DEPENDENCY_TAG_MATCHING[dep_type]

                # Remove tags that were converted to depend tags
                self.remove_dependencies(tag_name, existing_deps[dep_type].intersection(depend_tags))

                # Insert new tags
                remaining_deps = dependency_dict[dep_type] - existing_deps[dep_type]
                self.insert_new_packages(tag_name, remaining_deps - depend_tags)

            # Insert depends
            existing_depend = self.get_packages_by_tag('depend')
            self.insert_new_packages('depend', depend_tags - existing_depend)
        else:
            # Only insert depend tag when there is a BUILD, EXPORT *and* RUN dependency
            existing_triple = None
            end_deps = None
            for dep_type in DEPEND_TYPES:
                all_deps = dependency_dict[dep_type].union(existing_deps[dep_type])
                if existing_triple is None:
                    existing_triple = set(existing_deps[dep_type])
                    end_deps = set(all_deps)
                else:
                    existing_triple &= existing_deps[dep_type]
                    end_deps &= all_deps

            existing_depend = self.get_packages_by_tag('depend')
            depends_to_add = end_deps - existing_depend - existing_triple
            self.insert_new_packages('depend', depends_to_add)

            for dep_type in DEPEND_TYPES:
                tag_name = DEPENDENCY_TAG_MATCHING[dep_type]

                # Remove tags that were converted to depend tags
                self.remove_dependencies(tag_name, existing_deps[dep_type].intersection(depends_to_add))

                # Insert new tags
                remaining_deps = dependency_dict[dep_type] - existing_deps[dep_type]
                self.insert_new_packages(tag_name, remaining_deps - end_deps)

        if dependency_dict.get(DependencyType.TEST):
            test_depends = dependency_dict[DependencyType.TEST] - existing_deps[DependencyType.TEST]
            # Also remove build commands
            test_depends -= (existing_deps[DependencyType.BUILD] | dependency_dict[DependencyType.BUILD])
            self.insert_new_packages('test_depend', test_depends)

    def remove_dependencies(self, name, pkgs):
        for el in self.root.getElementsByTagName(name):
            pkg = el.childNodes[0].nodeValue
            if pkg in pkgs:
                self.remove_element(el)

    def remove_element(self, element):
        """Remove the given element AND the text element before it if it is just an indentation."""
        parent = element.parentNode
        assert parent
        index = parent.childNodes.index(element)
        if index > 0:
            previous = parent.childNodes[index - 1]
            if previous.nodeType == previous.TEXT_NODE and INDENT_PATTERN.match(previous.nodeValue):
                parent.removeChild(previous)
        parent.removeChild(element)
        self.changed = True

    def regenerate_contents(self):
        s = self.tree.toxml(self.tree.encoding)
        index = get_package_tag_index(s)
        s = self.header + s[index:] + '\n'
        return s
