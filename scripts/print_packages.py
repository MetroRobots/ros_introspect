#!/usr/bin/env python3

from ros_introspection.util import get_packages

for package in get_packages():
    print(package)
