# generated from rosidl_generator_py/resource/_idl.py.em
# with input from cola2_msgs:msg/WorldWaypointReq.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_WorldWaypointReq(type):
    """Metaclass of message 'WorldWaypointReq'."""

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
                'cola2_msgs.msg.WorldWaypointReq')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__world_waypoint_req
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__world_waypoint_req
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__world_waypoint_req
            cls._TYPE_SUPPORT = module.type_support_msg__msg__world_waypoint_req
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__world_waypoint_req

            from cola2_msgs.msg import Bool6Axis
            if Bool6Axis.__class__._TYPE_SUPPORT is None:
                Bool6Axis.__class__.__import_type_support__()

            from cola2_msgs.msg import GoalDescriptor
            if GoalDescriptor.__class__._TYPE_SUPPORT is None:
                GoalDescriptor.__class__.__import_type_support__()

            from cola2_msgs.msg import NED
            if NED.__class__._TYPE_SUPPORT is None:
                NED.__class__.__import_type_support__()

            from cola2_msgs.msg import RPY
            if RPY.__class__._TYPE_SUPPORT is None:
                RPY.__class__.__import_type_support__()

            from geometry_msgs.msg import Vector3
            if Vector3.__class__._TYPE_SUPPORT is None:
                Vector3.__class__.__import_type_support__()

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


class WorldWaypointReq(metaclass=Metaclass_WorldWaypointReq):
    """Message class 'WorldWaypointReq'."""

    __slots__ = [
        '_header',
        '_goal',
        '_altitude_mode',
        '_position',
        '_altitude',
        '_orientation',
        '_disable_axis',
        '_position_tolerance',
        '_orientation_tolerance',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'goal': 'cola2_msgs/GoalDescriptor',
        'altitude_mode': 'boolean',
        'position': 'cola2_msgs/NED',
        'altitude': 'float',
        'orientation': 'cola2_msgs/RPY',
        'disable_axis': 'cola2_msgs/Bool6Axis',
        'position_tolerance': 'geometry_msgs/Vector3',
        'orientation_tolerance': 'cola2_msgs/RPY',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['cola2_msgs', 'msg'], 'GoalDescriptor'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['cola2_msgs', 'msg'], 'NED'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['cola2_msgs', 'msg'], 'RPY'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['cola2_msgs', 'msg'], 'Bool6Axis'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['cola2_msgs', 'msg'], 'RPY'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from cola2_msgs.msg import GoalDescriptor
        self.goal = kwargs.get('goal', GoalDescriptor())
        self.altitude_mode = kwargs.get('altitude_mode', bool())
        from cola2_msgs.msg import NED
        self.position = kwargs.get('position', NED())
        self.altitude = kwargs.get('altitude', float())
        from cola2_msgs.msg import RPY
        self.orientation = kwargs.get('orientation', RPY())
        from cola2_msgs.msg import Bool6Axis
        self.disable_axis = kwargs.get('disable_axis', Bool6Axis())
        from geometry_msgs.msg import Vector3
        self.position_tolerance = kwargs.get('position_tolerance', Vector3())
        from cola2_msgs.msg import RPY
        self.orientation_tolerance = kwargs.get('orientation_tolerance', RPY())

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
        if self.goal != other.goal:
            return False
        if self.altitude_mode != other.altitude_mode:
            return False
        if self.position != other.position:
            return False
        if self.altitude != other.altitude:
            return False
        if self.orientation != other.orientation:
            return False
        if self.disable_axis != other.disable_axis:
            return False
        if self.position_tolerance != other.position_tolerance:
            return False
        if self.orientation_tolerance != other.orientation_tolerance:
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
    def goal(self):
        """Message field 'goal'."""
        return self._goal

    @goal.setter
    def goal(self, value):
        if __debug__:
            from cola2_msgs.msg import GoalDescriptor
            assert \
                isinstance(value, GoalDescriptor), \
                "The 'goal' field must be a sub message of type 'GoalDescriptor'"
        self._goal = value

    @builtins.property
    def altitude_mode(self):
        """Message field 'altitude_mode'."""
        return self._altitude_mode

    @altitude_mode.setter
    def altitude_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'altitude_mode' field must be of type 'bool'"
        self._altitude_mode = value

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
    def disable_axis(self):
        """Message field 'disable_axis'."""
        return self._disable_axis

    @disable_axis.setter
    def disable_axis(self, value):
        if __debug__:
            from cola2_msgs.msg import Bool6Axis
            assert \
                isinstance(value, Bool6Axis), \
                "The 'disable_axis' field must be a sub message of type 'Bool6Axis'"
        self._disable_axis = value

    @builtins.property
    def position_tolerance(self):
        """Message field 'position_tolerance'."""
        return self._position_tolerance

    @position_tolerance.setter
    def position_tolerance(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'position_tolerance' field must be a sub message of type 'Vector3'"
        self._position_tolerance = value

    @builtins.property
    def orientation_tolerance(self):
        """Message field 'orientation_tolerance'."""
        return self._orientation_tolerance

    @orientation_tolerance.setter
    def orientation_tolerance(self, value):
        if __debug__:
            from cola2_msgs.msg import RPY
            assert \
                isinstance(value, RPY), \
                "The 'orientation_tolerance' field must be a sub message of type 'RPY'"
        self._orientation_tolerance = value
