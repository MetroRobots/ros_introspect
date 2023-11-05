from ros_introspect.components.package_xml import INITIAL_TAGS, MBLOCK_TAGS, DEPEND_TAGS, FINAL_TAGS, PEOPLE_TAGS
from ros_introspect.components.package_xml import INDENT_PATTERN, FORMAT_3_HEADER
import collections

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
        print('\tUnsure of ordering for ' + name)
    return len(ordering)


def replace_package_set(manifest, source_tags, new_tag):
    """Replace all the elements with tags in source_tags with new elements with new_tag."""
    intersection = None
    for tag in source_tags:
        pkgs = set(manifest.get_packages_by_tag(tag))
        if intersection is None:
            intersection = pkgs
        else:
            intersection = intersection.intersection(pkgs)
    for tag in source_tags:
        manifest.remove_dependencies(tag, intersection)
    manifest.insert_new_packages(new_tag, intersection)


class PackageXML:
    def get_packages(self, mode='build'):
        keys = []
        if mode == 'build':
            keys.append('build_depend')
        if self.format == 1 and mode == 'run':
            keys.append('run_depend')
        if self.format >= 2 and mode != 'test':
            keys.append('depend')
            if mode == 'run':
                keys.append('exec_depend')
        if mode == 'test':
            keys.append('test_depend')
        pkgs = []
        for key in keys:
            pkgs += self.get_packages_by_tag(key)
        return set(pkgs)

    def get_child_indexes(self):
        """Return a dictionary based on which children span which indexes.

        The keys are the types of nodes in the xml (build_depend, maintainer, etc).
        The values are arrays marking the range of elements in the xml root that match that tag.

        For example, tags[build_depend] = [(5, 9), (11, 50)] means that elements [5, 9) and [11, 50) are
        either build_depend elements (or the strings between them)
        """
        tags = collections.defaultdict(list)
        i = 0
        current = None
        current_start = 0
        current_last = 0
        while i < len(self.root.childNodes):
            child = self.root.childNodes[i]
            if child.nodeType == child.TEXT_NODE:
                i += 1
                continue

            name = child.nodeName
            if name != current:
                if current:
                    tags[current].append((current_start, current_last))
                current_start = i
                current = name
            current_last = i
            i += 1
        if current:
            tags[current].append((current_start, current_last))
        return dict(tags)

    def get_insertion_index(self, tag, tag_value=None):
        """Return the index where to insert a new element with the given tag type.

        If there are already elements of that type, then either insert after the last matching element,
        or if the list is alphabetized, insert it in the correct place alphabetically using the tag_value.
        Otherwise, look at the existing elements, and find ones that are supposed to come the closest
        before the given tag, and insert after them. If none found, add at the end.
        """
        indexes = self.get_child_indexes()
        # If there are elements of this type already
        if tag in indexes:
            if len(indexes[tag]) == 1 and tag in DEPEND_TAGS:
                start, end = indexes[tag][0]
                tag_values = []
                my_index = start
                for i in range(start, end + 1):
                    child = self.root.childNodes[i]
                    if child.nodeType == child.TEXT_NODE:
                        continue
                    value = child.firstChild.data
                    tag_values.append(value)
                    if tag_value >= value:
                        my_index = i

                # If already sorted, and first_value is defined (meaning there are existing tags)
                if tag_values and sorted(tag_values) == tag_values:
                    # If it should go before the current first tag, we XXX
                    if tag_value <= tag_values[0]:
                        return my_index - 1

                    # If it should go before some existing tag
                    if tag_value <= tag_values[-1]:
                        return my_index

            # If all else fails, we insert the tag after the last matching tag
            return indexes[tag][-1][1]  # last match, end index

        # If no elements match this type, then find the right place to insert
        else:
            max_index = get_ordering_index(tag, whiny=False)
            best_tag = None
            best_index = None
            for tag in indexes:
                ni = get_ordering_index(tag, whiny=False)
                if ni >= max_index:
                    # This tag should appear after our tag
                    continue

                if best_tag is None or ni > best_index or indexes[tag][-1] > indexes[best_tag][-1]:
                    best_tag = tag
                    best_index = ni

            if best_tag is None:
                return len(self.root.childNodes)
            else:
                return indexes[best_tag][-1][1]

    def insert_new_tag_inside_another(self, parent, tag, depth=2):
        all_elements = []
        all_elements.append(self.create_new_tab_element(depth))
        all_elements.append(tag)

        if len(parent.childNodes) == 0:
            parent.childNodes = all_elements + [self.create_new_tab_element()]
        else:
            parent.childNodes = parent.childNodes[:-1] + all_elements + parent.childNodes[-1:]
        self.changed = True

    def insert_new_packages(self, tag, values):
        for pkg in sorted(values):
            print('\tInserting %s: %s' % (tag, pkg))
            node = self.tree.createElement(tag)
            node.appendChild(self.tree.createTextNode(pkg))
            self.insert_new_tag(node)

    def add_packages(self, build_depends, run_depends, test_depends=None, prefer_depend_tag=True):
        if self.format == 1:
            run_depends.update(build_depends)
        existing_build = self.get_packages('build')
        existing_run = self.get_packages('run')
        build_depends = build_depends - existing_build
        run_depends = run_depends - existing_run
        # Todo: Just insert run depend
        if self.format == 1:
            self.insert_new_packages('build_depend', build_depends)
            self.insert_new_packages('run_depend', run_depends)
        elif prefer_depend_tag:
            depend_tags = build_depends.union(run_depends)

            # Remove tags that overlap with new depends
            self.remove_dependencies('build_depend', existing_build.intersection(depend_tags))
            self.remove_dependencies('exec_depend', existing_run.intersection(depend_tags))

            # Insert depends
            self.insert_new_packages('depend', depend_tags)
        else:
            both = build_depends.intersection(run_depends)
            self.insert_new_packages('depend', both)
            self.insert_new_packages('build_depend', build_depends - both)
            self.insert_new_packages('exec_depend', build_depends - both - existing_run)
            self.insert_new_packages('exec_depend', run_depends - both)

        if test_depends is not None and len(test_depends) > 0:
            existing_test = self.get_packages('test')
            test_depends = set(test_depends) - existing_build - build_depends - existing_test
            self.insert_new_packages('test_depend', test_depends)

    def remove_element(self, element):
        """Remove the given element AND the text element before it if it is just an indentation."""
        parent = element.parentNode
        if not parent:
            return
        index = parent.childNodes.index(element)
        if index > 0:
            previous = parent.childNodes[index - 1]
            if previous.nodeType == previous.TEXT_NODE and INDENT_PATTERN.match(previous.nodeValue):
                parent.removeChild(previous)
        parent.removeChild(element)
        self.changed = True

    def remove_dependencies(self, name, pkgs):
        for el in self.root.getElementsByTagName(name):
            pkg = el.childNodes[0].nodeValue
            if pkg in pkgs:
                self.remove_element(el)

    def update_people(self, target_name, target_email=None, search_name=None, search_email=None):
        for el in self.get_elements_by_tags(PEOPLE_TAGS):
            name = el.childNodes[0].nodeValue
            email = el.getAttribute('email') if el.hasAttribute('email') else ''
            if (search_name is None or name == search_name) and (search_email is None or email == search_email):
                el.childNodes[0].nodeValue = target_name
                if target_email:
                    el.setAttribute('email', target_email)
                self.changed = True

    def set_license(self, license_str):
        el = self.get_license_element()
        if license != el.childNodes[0].nodeValue:
            el.childNodes[0].nodeValue = license_str
            self.changed = True

    def get_plugin_xmls(self):
        """Return a mapping from the package name to a list of the relative path(s) for the plugin xml(s)."""
        xmls = collections.defaultdict(list)
        export = self.root.getElementsByTagName('export')
        if len(export) == 0:
            return xmls
        for ex in export:
            for n in ex.childNodes:
                if n.nodeType == self.root.ELEMENT_NODE:
                    plugin = n.getAttribute('plugin').replace('${prefix}/', '')
                    xmls[n.nodeName].append(plugin)
        return xmls

    def get_export_tag(self):
        """Get the export tag. Create it if it doesn't exist."""
        export_tags = self.root.getElementsByTagName('export')
        if len(export_tags) == 0:
            export_tag = self.tree.createElement('export')
            self.insert_new_tag(export_tag)
            return export_tag
        else:
            return export_tags[0]

    def add_plugin_export(self, pkg_name, xml_path):
        """Add the plugin configuration if not found. Add export tag as needed. Return the surrounding export tag."""
        ex_tag = self.get_export_tag()

        attr = '${prefix}/' + xml_path
        for tag in ex_tag.childNodes:
            if tag.nodeName != pkg_name:
                continue
            plugin = tag.attributes.get('plugin')
            if plugin and plugin.value == attr:
                return

        pe = self.tree.createElement(pkg_name)
        pe.setAttribute('plugin', attr)
        self.insert_new_tag_inside_another(ex_tag, pe)
        return ex_tag

    def upgrade(self, new_format=2, quiet=True):
        if self.format == new_format:
            if not quiet:
                print('%s already in format %d!' % (self.name, self.format))
            return

        if new_format not in [2, 3]:
            raise RuntimeError('Unknown PackageXML version: ' + repr(new_format))

        if self.format == 1:
            if not quiet:
                print('Converting {} from version {} to 2'.format(self.name, self.format))
            self._format = 2
            self.root.setAttribute('format', '2')
            replace_package_set(self, ['build_depend', 'run_depend'], 'depend')
            replace_package_set(self, ['run_depend'], 'exec_depend')

        if new_format == 3:
            if not quiet:
                print('Converting {} from version {} to 3'.format(self.name, self.format))
            self._format = 3
            self.root.setAttribute('format', '3')
            self.header = FORMAT_3_HEADER

        self.changed = True
