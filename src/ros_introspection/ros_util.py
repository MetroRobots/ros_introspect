from ament_index_python.packages import get_package_share_path
from ament_index_python import get_packages_with_prefixes

from rosidl_runtime_py import get_action_interfaces
from rosidl_runtime_py import get_message_interfaces
from rosidl_runtime_py import get_service_interfaces


extension_map = {
    '.msg': get_message_interfaces,
    '.srv': get_service_interfaces,
    '.action': get_action_interfaces,
}


def get_package_path(package_name):
    return get_package_share_path(package_name)


def get_package_file(package_name, path):
    folder = get_package_path(package_name)
    full_path = folder / path
    if full_path.exists():
        return full_path


def list_packages():
    return list(get_packages_with_prefixes().keys())


def list_interfaces(package_name, extension='.msg'):
    results = extension_map[extension]([package_name])
    for pkg, interfaces in results.items():
        for gen_key in interfaces:
            gen = gen_key.split('/')[-1]
            yield pkg, gen
