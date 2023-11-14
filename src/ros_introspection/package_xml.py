import collections
from ros_introspect.components.package_xml import DEPEND_TAGS, PEOPLE_TAGS
from ros_introspect.components.package_xml import FORMAT_3_HEADER
from x import get_ordering_index


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
        if self.xml_format == new_format:
            if not quiet:
                print('%s already in format %d!' % (self.name, self.xml_format))
            return

        if new_format not in [2, 3]:
            raise RuntimeError('Unknown PackageXML version: ' + repr(new_format))

        if self.xml_format == 1:
            if not quiet:
                print('Converting {} from version {} to 2'.format(self.name, self.xml_format))
            self._format = 2
            self.root.setAttribute('format', '2')
            replace_package_set(self, ['build_depend', 'run_depend'], 'depend')
            replace_package_set(self, ['run_depend'], 'exec_depend')

        if new_format == 3:
            if not quiet:
                print('Converting {} from version {} to 3'.format(self.name, self.xml_format))
            self._format = 3
            self.root.setAttribute('format', '3')
            self.header = FORMAT_3_HEADER

        self.changed = True
