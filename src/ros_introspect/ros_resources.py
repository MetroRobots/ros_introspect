from betsy_ros import get_package_names, list_interfaces


class ROSResources(object):
    """Package wrapping the names of ROS resources to avoid ROS dependencies"""

    def __init__(self):
        self.packages = set()
        self.messages = set()
        self.services = set()
        self.actions = set()

    # Singleton
    def __new__(cls):
        if not hasattr(cls, 'instance'):
            cls.instance = super().__new__(cls)
        return cls.instance

    def load_from_ros(self):
        self.packages |= get_package_names()
        for interface in list_interfaces():
            if interface.type == 'msg':
                self.messages.add(interface)
            elif interface.type == 'srv':
                self.services.add(interface)
            else:
                self.actions.add(interface)

    def is_package(self, pkg):
        return pkg in self.packages

    def is_message(self, pkg, msg):
        return (pkg, msg) in self.messages

    def is_service(self, pkg, srv):
        return (pkg, srv) in self.services

    def is_action(self, pkg, action):
        return (pkg, action) in self.actions
