import collections
from ros_introspect.components.package_xml import PEOPLE_TAGS
from ros_introspect.components.package_xml import FORMAT_3_HEADER
from clean_ros.cleaners.package_xml import replace_package_set


class PackageXML:
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
