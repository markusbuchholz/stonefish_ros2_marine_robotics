# generated from rosidl_generator_py/resource/_idl.py.em
# with input from cola2_msgs:msg/NavSts.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_NavSts(type):
    """Metaclass of message 'NavSts'."""

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
            module = import_type_support('cola2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'cola2_msgs.msg.NavSts')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__nav_sts
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__nav_sts
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__nav_sts
            cls._TYPE_SUPPORT = module.type_support_msg__msg__nav_sts
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__nav_sts

            from cola2_msgs.msg import DecimalLatLon
            if DecimalLatLon.__class__._TYPE_SUPPORT is None:
                DecimalLatLon.__class__.__import_type_support__()

            from cola2_msgs.msg import NED
            if NED.__class__._TYPE_SUPPORT is None:
                NED.__class__.__import_type_support__()

            from cola2_msgs.msg import RPY
            if RPY.__class__._TYPE_SUPPORT is None:
                RPY.__class__.__import_type_support__()

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

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


class NavSts(metaclass=Metaclass_NavSts):
    """Message class 'NavSts'."""

    __slots__ = [
        '_header',
        '_global_position',
        '_origin',
        '_position',
        '_altitude',
        '_body_velocity',
        '_orientation',
        '_orientation_rate',
        '_position_variance',
        '_orientation_variance',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'global_position': 'cola2_msgs/DecimalLatLon',
        'origin': 'cola2_msgs/DecimalLatLon',
        'position': 'cola2_msgs/NED',
        'altitude': 'float',
        'body_velocity': 'geometry_msgs/Point',
        'orientation': 'cola2_msgs/RPY',
        'orientation_rate': 'cola2_msgs/RPY',
        'position_variance': 'cola2_msgs/NED',
        'orientation_variance': 'cola2_msgs/RPY',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['cola2_msgs', 'msg'], 'DecimalLatLon'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['cola2_msgs', 'msg'], 'DecimalLatLon'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['cola2_msgs', 'msg'], 'NED'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['cola2_msgs', 'msg'], 'RPY'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['cola2_msgs', 'msg'], 'RPY'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['cola2_msgs', 'msg'], 'NED'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['cola2_msgs', 'msg'], 'RPY'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from cola2_msgs.msg import DecimalLatLon
        self.global_position = kwargs.get('global_position', DecimalLatLon())
        from cola2_msgs.msg import DecimalLatLon
        self.origin = kwargs.get('origin', DecimalLatLon())
        from cola2_msgs.msg import NED
        self.position = kwargs.get('position', NED())
        self.altitude = kwargs.get('altitude', float())
        from geometry_msgs.msg import Point
        self.body_velocity = kwargs.get('body_velocity', Point())
        from cola2_msgs.msg import RPY
        self.orientation = kwargs.get('orientation', RPY())
        from cola2_msgs.msg import RPY
        self.orientation_rate = kwargs.get('orientation_rate', RPY())
        from cola2_msgs.msg import NED
        self.position_variance = kwargs.get('position_variance', NED())
        from cola2_msgs.msg import RPY
        self.orientation_variance = kwargs.get('orientation_variance', RPY())

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
        if self.global_position != other.global_position:
            return False
        if self.origin != other.origin:
            return False
        if self.position != other.position:
            return False
        if self.altitude != other.altitude:
            return False
        if self.body_velocity != other.body_velocity:
            return False
        if self.orientation != other.orientation:
            return False
        if self.orientation_rate != other.orientation_rate:
            return False
        if self.position_variance != other.position_variance:
            return False
        if self.orientation_variance != other.orientation_variance:
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
    def global_position(self):
        """Message field 'global_position'."""
        return self._global_position

    @global_position.setter
    def global_position(self, value):
        if __debug__:
            from cola2_msgs.msg import DecimalLatLon
            assert \
                isinstance(value, DecimalLatLon), \
                "The 'global_position' field must be a sub message of type 'DecimalLatLon'"
        self._global_position = value

    @builtins.property
    def origin(self):
        """Message field 'origin'."""
        return self._origin

    @origin.setter
    def origin(self, value):
        if __debug__:
            from cola2_msgs.msg import DecimalLatLon
            assert \
                isinstance(value, DecimalLatLon), \
                "The 'origin' field must be a sub message of type 'DecimalLatLon'"
        self._origin = value

    @builtins.property
    def position(self):
        """Message field 'position'."""
        return self._position

    @position.setter
    def position(self, value):
        if __debug__:
            from cola2_msgs.msg import NED
            assert \
                isinstance(value, NED), \
                "The 'position' field must be a sub message of type 'NED'"
        self._position = value

    @builtins.property
    def altitude(self):
        """Message field 'altitude'."""
        return self._altitude

    @altitude.setter
    def altitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'altitude' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'altitude' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._altitude = value

    @builtins.property
    def body_velocity(self):
        """Message field 'body_velocity'."""
        return self._body_velocity

    @body_velocity.setter
    def body_velocity(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'body_velocity' field must be a sub message of type 'Point'"
        self._body_velocity = value

    @builtins.property
    def orientation(self):
        """Message field 'orientation'."""
        return self._orientation

    @orientation.setter
    def orientation(self, value):
        if __debug__:
            from cola2_msgs.msg import RPY
            assert \
                isinstance(value, RPY), \
                "The 'orientation' field must be a sub message of type 'RPY'"
        self._orientation = value

    @builtins.property
    def orientation_rate(self):
        """Message field 'orientation_rate'."""
        return self._orientation_rate

    @orientation_rate.setter
    def orientation_rate(self, value):
        if __debug__:
            from cola2_msgs.msg import RPY
            assert \
                isinstance(value, RPY), \
                "The 'orientation_rate' field must be a sub message of type 'RPY'"
        self._orientation_rate = value

    @builtins.property
    def position_variance(self):
        """Message field 'position_variance'."""
        return self._position_variance

    @position_variance.setter
    def position_variance(self, value):
        if __debug__:
            from cola2_msgs.msg import NED
            assert \
                isinstance(value, NED), \
                "The 'position_variance' field must be a sub message of type 'NED'"
        self._position_variance = value

    @builtins.property
    def orientation_variance(self):
        """Message field 'orientation_variance'."""
        return self._orientation_variance

    @orientation_variance.setter
    def orientation_variance(self, value):
        if __debug__:
            from cola2_msgs.msg import RPY
            assert \
                isinstance(value, RPY), \
                "The 'orientation_variance' field must be a sub message of type 'RPY'"
        self._orientation_variance = value
