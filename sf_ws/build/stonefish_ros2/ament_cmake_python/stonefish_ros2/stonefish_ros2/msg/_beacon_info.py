# generated from rosidl_generator_py/resource/_idl.py.em
# with input from stonefish_ros2:msg/BeaconInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BeaconInfo(type):
    """Metaclass of message 'BeaconInfo'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('stonefish_ros2')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'stonefish_ros2.msg.BeaconInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__beacon_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__beacon_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__beacon_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__beacon_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__beacon_info

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

            from geometry_msgs.msg import Quaternion
            if Quaternion.__class__._TYPE_SUPPORT is None:
                Quaternion.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class BeaconInfo(metaclass=Metaclass_BeaconInfo):
    """Message class 'BeaconInfo'."""

    __slots__ = [
        '_header',
        '_beacon_id',
        '_range',
        '_azimuth',
        '_elevation',
        '_relative_position',
        '_local_orientation',
        '_local_depth',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'beacon_id': 'uint8',
        'range': 'float',
        'azimuth': 'float',
        'elevation': 'float',
        'relative_position': 'geometry_msgs/Point',
        'local_orientation': 'geometry_msgs/Quaternion',
        'local_depth': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Quaternion'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.beacon_id = kwargs.get('beacon_id', int())
        self.range = kwargs.get('range', float())
        self.azimuth = kwargs.get('azimuth', float())
        self.elevation = kwargs.get('elevation', float())
        from geometry_msgs.msg import Point
        self.relative_position = kwargs.get('relative_position', Point())
        from geometry_msgs.msg import Quaternion
        self.local_orientation = kwargs.get('local_orientation', Quaternion())
        self.local_depth = kwargs.get('local_depth', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.beacon_id != other.beacon_id:
            return False
        if self.range != other.range:
            return False
        if self.azimuth != other.azimuth:
            return False
        if self.elevation != other.elevation:
            return False
        if self.relative_position != other.relative_position:
            return False
        if self.local_orientation != other.local_orientation:
            return False
        if self.local_depth != other.local_depth:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def beacon_id(self):
        """Message field 'beacon_id'."""
        return self._beacon_id

    @beacon_id.setter
    def beacon_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'beacon_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'beacon_id' field must be an unsigned integer in [0, 255]"
        self._beacon_id = value

    @builtins.property  # noqa: A003
    def range(self):  # noqa: A003
        """Message field 'range'."""
        return self._range

    @range.setter  # noqa: A003
    def range(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'range' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'range' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._range = value

    @builtins.property
    def azimuth(self):
        """Message field 'azimuth'."""
        return self._azimuth

    @azimuth.setter
    def azimuth(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'azimuth' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'azimuth' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._azimuth = value

    @builtins.property
    def elevation(self):
        """Message field 'elevation'."""
        return self._elevation

    @elevation.setter
    def elevation(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'elevation' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'elevation' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._elevation = value

    @builtins.property
    def relative_position(self):
        """Message field 'relative_position'."""
        return self._relative_position

    @relative_position.setter
    def relative_position(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'relative_position' field must be a sub message of type 'Point'"
        self._relative_position = value

    @builtins.property
    def local_orientation(self):
        """Message field 'local_orientation'."""
        return self._local_orientation

    @local_orientation.setter
    def local_orientation(self, value):
        if __debug__:
            from geometry_msgs.msg import Quaternion
            assert \
                isinstance(value, Quaternion), \
                "The 'local_orientation' field must be a sub message of type 'Quaternion'"
        self._local_orientation = value

    @builtins.property
    def local_depth(self):
        """Message field 'local_depth'."""
        return self._local_depth

    @local_depth.setter
    def local_depth(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'local_depth' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'local_depth' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._local_depth = value
