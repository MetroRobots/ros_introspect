import pytest

from ros_introspect.util import convert_to_caps_notation, convert_to_underscore_notation

test_strings = [('ros', 'Ros', 'ros'),
                ('David Lu', 'DavidLu', 'david_lu'),
                ('Quick-brown Dog', 'QuickBrownDog', 'quick_brown_dog'),
                ('Package XML', 'PackageXml', 'package_xml'),
                ]


@pytest.mark.parametrize('input_s, caps_s, underscore_s', test_strings, ids=[k[0] for k in test_strings])
def test_forward_backward(input_s, caps_s, underscore_s):

    assert convert_to_underscore_notation(input_s) == underscore_s
    assert convert_to_caps_notation(input_s) == caps_s

    assert convert_to_caps_notation(caps_s) == caps_s
    assert convert_to_underscore_notation(caps_s) == underscore_s

    assert convert_to_caps_notation(underscore_s) == caps_s
    assert convert_to_underscore_notation(underscore_s) == underscore_s
