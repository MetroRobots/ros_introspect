from betsy_ros import get_package_names, list_interfaces


def match_package_and_name(interfaces, pkg, name):
    for interface in interfaces:
        if interface.package == pkg and interface.name == name:
            return interface


class ROSResources(object):
    """Package wrapping the names of ROS resources to avoid ROS dependencies"""

    # Singleton
    def __new__(cls):
        if not hasattr(cls, 'instance'):
            cls.instance = super().__new__(cls)
            cls.instance.packages = set()
            cls.instance.messages = set()
            cls.instance.services = set()
            cls.instance.actions = set()
        return cls.instance

    @classmethod
    def get(cls):
        return cls()

    def load_from_ros(self):
        self.packages |= get_package_names()
        for interface in list_interfaces():
            self.add_interface(interface)

    def add_interface(self, interface):
        if interface.type == 'msg':
            self.messages.add(interface)
        elif interface.type == 'srv':
            self.services.add(interface)
        else:
            self.actions.add(interface)

    def is_package(self, pkg):
        return pkg in self.packages

    def is_message(self, pkg, msg):
        return match_package_and_name(self.messages, pkg, msg)

    def is_service(self, pkg, srv):
        return match_package_and_name(self.services, pkg, srv)

    def is_action(self, pkg, action):
        return match_package_and_name(self.actions, pkg, action)
