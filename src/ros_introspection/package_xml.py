from ros_introspect.components.package_xml import PEOPLE_TAGS
from ros_introspect.components.package_xml import FORMAT_3_HEADER
from clean_ros.cleaners.package_xml import replace_package_set


class PackageXML:
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
