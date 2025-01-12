# generated from rosidl_generator_py/resource/_idl.py.em
# with input from cola2_msgs:action/Pilot.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Pilot_Goal(type):
    """Metaclass of message 'Pilot_Goal'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'DEPTH': 0,
        'ALTITUDE': 1,
        'BOTH': 2,
        'SECTION': 0,
        'ANCHOR': 1,
        'HOLONOMIC_KEEP_POSITION': 2,
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
                'cola2_msgs.action.Pilot_Goal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__pilot__goal
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__pilot__goal
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__pilot__goal
            cls._TYPE_SUPPORT = module.type_support_msg__action__pilot__goal
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__pilot__goal

            from cola2_msgs.msg import GoalDescriptor
            if GoalDescriptor.__class__._TYPE_SUPPORT is None:
                GoalDescriptor.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'DEPTH': cls.__constants['DEPTH'],
            'ALTITUDE': cls.__constants['ALTITUDE'],
            'BOTH': cls.__constants['BOTH'],
            'SECTION': cls.__constants['SECTION'],
            'ANCHOR': cls.__constants['ANCHOR'],
            'HOLONOMIC_KEEP_POSITION': cls.__constants['HOLONOMIC_KEEP_POSITION'],
        }

    @property
    def DEPTH(self):
        """Message constant 'DEPTH'."""
        return Metaclass_Pilot_Goal.__constants['DEPTH']

    @property
    def ALTITUDE(self):
        """Message constant 'ALTITUDE'."""
        return Metaclass_Pilot_Goal.__constants['ALTITUDE']

    @property
    def BOTH(self):
        """Message constant 'BOTH'."""
        return Metaclass_Pilot_Goal.__constants['BOTH']

    @property
    def SECTION(self):
        """Message constant 'SECTION'."""
        return Metaclass_Pilot_Goal.__constants['SECTION']

    @property
    def ANCHOR(self):
        """Message constant 'ANCHOR'."""
        return Metaclass_Pilot_Goal.__constants['ANCHOR']

    @property
    def HOLONOMIC_KEEP_POSITION(self):
        """Message constant 'HOLONOMIC_KEEP_POSITION'."""
        return Metaclass_Pilot_Goal.__constants['HOLONOMIC_KEEP_POSITION']


class Pilot_Goal(metaclass=Metaclass_Pilot_Goal):
    """
    Message class 'Pilot_Goal'.

    Constants:
      DEPTH
      ALTITUDE
      BOTH
      SECTION
      ANCHOR
      HOLONOMIC_KEEP_POSITION
    """

    __slots__ = [
        '_initial_latitude',
        '_initial_longitude',
        '_initial_depth',
        '_final_latitude',
        '_final_longitude',
        '_final_depth',
        '_final_yaw',
        '_final_altitude',
        '_heave_mode',
        '_surge_velocity',
        '_tolerance_xy',
        '_timeout',
        '_controller_type',
        '_goal',
    ]

    _fields_and_field_types = {
        'initial_latitude': 'double',
        'initial_longitude': 'double',
        'initial_depth': 'double',
        'final_latitude': 'double',
        'final_longitude': 'double',
        'final_depth': 'double',
        'final_yaw': 'double',
        'final_altitude': 'double',
        'heave_mode': 'uint64',
        'surge_velocity': 'double',
        'tolerance_xy': 'double',
        'timeout': 'double',
        'controller_type': 'uint64',
        'goal': 'cola2_msgs/GoalDescriptor',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['cola2_msgs', 'msg'], 'GoalDescriptor'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.initial_latitude = kwargs.get('initial_latitude', float())
        self.initial_longitude = kwargs.get('initial_longitude', float())
        self.initial_depth = kwargs.get('initial_depth', float())
        self.final_latitude = kwargs.get('final_latitude', float())
        self.final_longitude = kwargs.get('final_longitude', float())
        self.final_depth = kwargs.get('final_depth', float())
        self.final_yaw = kwargs.get('final_yaw', float())
        self.final_altitude = kwargs.get('final_altitude', float())
        self.heave_mode = kwargs.get('heave_mode', int())
        self.surge_velocity = kwargs.get('surge_velocity', float())
        self.tolerance_xy = kwargs.get('tolerance_xy', float())
        self.timeout = kwargs.get('timeout', float())
        self.controller_type = kwargs.get('controller_type', int())
        from cola2_msgs.msg import GoalDescriptor
        self.goal = kwargs.get('goal', GoalDescriptor())

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
        if self.initial_latitude != other.initial_latitude:
            return False
        if self.initial_longitude != other.initial_longitude:
            return False
        if self.initial_depth != other.initial_depth:
            return False
        if self.final_latitude != other.final_latitude:
            return False
        if self.final_longitude != other.final_longitude:
            return False
        if self.final_depth != other.final_depth:
            return False
        if self.final_yaw != other.final_yaw:
            return False
        if self.final_altitude != other.final_altitude:
            return False
        if self.heave_mode != other.heave_mode:
            return False
        if self.surge_velocity != other.surge_velocity:
            return False
        if self.tolerance_xy != other.tolerance_xy:
            return False
        if self.timeout != other.timeout:
            return False
        if self.controller_type != other.controller_type:
            return False
        if self.goal != other.goal:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def initial_latitude(self):
        """Message field 'initial_latitude'."""
        return self._initial_latitude

    @initial_latitude.setter
    def initial_latitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'initial_latitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'initial_latitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._initial_latitude = value

    @builtins.property
    def initial_longitude(self):
        """Message field 'initial_longitude'."""
        return self._initial_longitude

    @initial_longitude.setter
    def initial_longitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'initial_longitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'initial_longitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._initial_longitude = value

    @builtins.property
    def initial_depth(self):
        """Message field 'initial_depth'."""
        return self._initial_depth

    @initial_depth.setter
    def initial_depth(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'initial_depth' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'initial_depth' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._initial_depth = value

    @builtins.property
    def final_latitude(self):
        """Message field 'final_latitude'."""
        return self._final_latitude

    @final_latitude.setter
    def final_latitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'final_latitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'final_latitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._final_latitude = value

    @builtins.property
    def final_longitude(self):
        """Message field 'final_longitude'."""
        return self._final_longitude

    @final_longitude.setter
    def final_longitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'final_longitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'final_longitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._final_longitude = value

    @builtins.property
    def final_depth(self):
        """Message field 'final_depth'."""
        return self._final_depth

    @final_depth.setter
    def final_depth(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'final_depth' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'final_depth' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._final_depth = value

    @builtins.property
    def final_yaw(self):
        """Message field 'final_yaw'."""
        return self._final_yaw

    @final_yaw.setter
    def final_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'final_yaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'final_yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._final_yaw = value

    @builtins.property
    def final_altitude(self):
        """Message field 'final_altitude'."""
        return self._final_altitude

    @final_altitude.setter
    def final_altitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'final_altitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'final_altitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._final_altitude = value

    @builtins.property
    def heave_mode(self):
        """Message field 'heave_mode'."""
        return self._heave_mode

    @heave_mode.setter
    def heave_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'heave_mode' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'heave_mode' field must be an unsigned integer in [0, 18446744073709551615]"
        self._heave_mode = value

    @builtins.property
    def surge_velocity(self):
        """Message field 'surge_velocity'."""
        return self._surge_velocity

    @surge_velocity.setter
    def surge_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'surge_velocity' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'surge_velocity' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._surge_velocity = value

    @builtins.property
    def tolerance_xy(self):
        """Message field 'tolerance_xy'."""
        return self._tolerance_xy

    @tolerance_xy.setter
    def tolerance_xy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tolerance_xy' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'tolerance_xy' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._tolerance_xy = value

    @builtins.property
    def timeout(self):
        """Message field 'timeout'."""
        return self._timeout

    @timeout.setter
    def timeout(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'timeout' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'timeout' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._timeout = value

    @builtins.property
    def controller_type(self):
        """Message field 'controller_type'."""
        return self._controller_type

    @controller_type.setter
    def controller_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'controller_type' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'controller_type' field must be an unsigned integer in [0, 18446744073709551615]"
        self._controller_type = value

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


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Pilot_Result(type):
    """Metaclass of message 'Pilot_Result'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'SUCCESS': 0,
        'TIMEOUT': 1,
        'FAILURE': 2,
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
                'cola2_msgs.action.Pilot_Result')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__pilot__result
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__pilot__result
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__pilot__result
            cls._TYPE_SUPPORT = module.type_support_msg__action__pilot__result
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__pilot__result

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'SUCCESS': cls.__constants['SUCCESS'],
            'TIMEOUT': cls.__constants['TIMEOUT'],
            'FAILURE': cls.__constants['FAILURE'],
        }

    @property
    def SUCCESS(self):
        """Message constant 'SUCCESS'."""
        return Metaclass_Pilot_Result.__constants['SUCCESS']

    @property
    def TIMEOUT(self):
        """Message constant 'TIMEOUT'."""
        return Metaclass_Pilot_Result.__constants['TIMEOUT']

    @property
    def FAILURE(self):
        """Message constant 'FAILURE'."""
        return Metaclass_Pilot_Result.__constants['FAILURE']


class Pilot_Result(metaclass=Metaclass_Pilot_Result):
    """
    Message class 'Pilot_Result'.

    Constants:
      SUCCESS
      TIMEOUT
      FAILURE
    """

    __slots__ = [
        '_state',
    ]

    _fields_and_field_types = {
        'state': 'uint64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.state = kwargs.get('state', int())

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
        if self.state != other.state:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'state' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'state' field must be an unsigned integer in [0, 18446744073709551615]"
        self._state = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_Pilot_Feedback(type):
    """Metaclass of message 'Pilot_Feedback'."""

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
                'cola2_msgs.action.Pilot_Feedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__pilot__feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__pilot__feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__pilot__feedback
            cls._TYPE_SUPPORT = module.type_support_msg__action__pilot__feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__pilot__feedback

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Pilot_Feedback(metaclass=Metaclass_Pilot_Feedback):
    """Message class 'Pilot_Feedback'."""

    __slots__ = [
        '_distance_to_end',
        '_cross_track_error',
    ]

    _fields_and_field_types = {
        'distance_to_end': 'double',
        'cross_track_error': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.distance_to_end = kwargs.get('distance_to_end', float())
        self.cross_track_error = kwargs.get('cross_track_error', float())

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
        if self.distance_to_end != other.distance_to_end:
            return False
        if self.cross_track_error != other.cross_track_error:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def distance_to_end(self):
        """Message field 'distance_to_end'."""
        return self._distance_to_end

    @distance_to_end.setter
    def distance_to_end(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'distance_to_end' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'distance_to_end' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._distance_to_end = value

    @builtins.property
    def cross_track_error(self):
        """Message field 'cross_track_error'."""
        return self._cross_track_error

    @cross_track_error.setter
    def cross_track_error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cross_track_error' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'cross_track_error' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._cross_track_error = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Pilot_SendGoal_Request(type):
    """Metaclass of message 'Pilot_SendGoal_Request'."""

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
                'cola2_msgs.action.Pilot_SendGoal_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__pilot__send_goal__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__pilot__send_goal__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__pilot__send_goal__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__pilot__send_goal__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__pilot__send_goal__request

            from cola2_msgs.action import Pilot
            if Pilot.Goal.__class__._TYPE_SUPPORT is None:
                Pilot.Goal.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Pilot_SendGoal_Request(metaclass=Metaclass_Pilot_SendGoal_Request):
    """Message class 'Pilot_SendGoal_Request'."""

    __slots__ = [
        '_goal_id',
        '_goal',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'goal': 'cola2_msgs/Pilot_Goal',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['cola2_msgs', 'action'], 'Pilot_Goal'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from cola2_msgs.action._pilot import Pilot_Goal
        self.goal = kwargs.get('goal', Pilot_Goal())

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
        if self.goal_id != other.goal_id:
            return False
        if self.goal != other.goal:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def goal(self):
        """Message field 'goal'."""
        return self._goal

    @goal.setter
    def goal(self, value):
        if __debug__:
            from cola2_msgs.action._pilot import Pilot_Goal
            assert \
                isinstance(value, Pilot_Goal), \
                "The 'goal' field must be a sub message of type 'Pilot_Goal'"
        self._goal = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Pilot_SendGoal_Response(type):
    """Metaclass of message 'Pilot_SendGoal_Response'."""

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
                'cola2_msgs.action.Pilot_SendGoal_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__pilot__send_goal__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__pilot__send_goal__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__pilot__send_goal__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__pilot__send_goal__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__pilot__send_goal__response

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Pilot_SendGoal_Response(metaclass=Metaclass_Pilot_SendGoal_Response):
    """Message class 'Pilot_SendGoal_Response'."""

    __slots__ = [
        '_accepted',
        '_stamp',
    ]

    _fields_and_field_types = {
        'accepted': 'boolean',
        'stamp': 'builtin_interfaces/Time',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.accepted = kwargs.get('accepted', bool())
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())

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
        if self.accepted != other.accepted:
            return False
        if self.stamp != other.stamp:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def accepted(self):
        """Message field 'accepted'."""
        return self._accepted

    @accepted.setter
    def accepted(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'accepted' field must be of type 'bool'"
        self._accepted = value

    @builtins.property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value


class Metaclass_Pilot_SendGoal(type):
    """Metaclass of service 'Pilot_SendGoal'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('cola2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'cola2_msgs.action.Pilot_SendGoal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__pilot__send_goal

            from cola2_msgs.action import _pilot
            if _pilot.Metaclass_Pilot_SendGoal_Request._TYPE_SUPPORT is None:
                _pilot.Metaclass_Pilot_SendGoal_Request.__import_type_support__()
            if _pilot.Metaclass_Pilot_SendGoal_Response._TYPE_SUPPORT is None:
                _pilot.Metaclass_Pilot_SendGoal_Response.__import_type_support__()


class Pilot_SendGoal(metaclass=Metaclass_Pilot_SendGoal):
    from cola2_msgs.action._pilot import Pilot_SendGoal_Request as Request
    from cola2_msgs.action._pilot import Pilot_SendGoal_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Pilot_GetResult_Request(type):
    """Metaclass of message 'Pilot_GetResult_Request'."""

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
                'cola2_msgs.action.Pilot_GetResult_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__pilot__get_result__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__pilot__get_result__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__pilot__get_result__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__pilot__get_result__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__pilot__get_result__request

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Pilot_GetResult_Request(metaclass=Metaclass_Pilot_GetResult_Request):
    """Message class 'Pilot_GetResult_Request'."""

    __slots__ = [
        '_goal_id',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())

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
        if self.goal_id != other.goal_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Pilot_GetResult_Response(type):
    """Metaclass of message 'Pilot_GetResult_Response'."""

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
                'cola2_msgs.action.Pilot_GetResult_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__pilot__get_result__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__pilot__get_result__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__pilot__get_result__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__pilot__get_result__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__pilot__get_result__response

            from cola2_msgs.action import Pilot
            if Pilot.Result.__class__._TYPE_SUPPORT is None:
                Pilot.Result.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Pilot_GetResult_Response(metaclass=Metaclass_Pilot_GetResult_Response):
    """Message class 'Pilot_GetResult_Response'."""

    __slots__ = [
        '_status',
        '_result',
    ]

    _fields_and_field_types = {
        'status': 'int8',
        'result': 'cola2_msgs/Pilot_Result',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['cola2_msgs', 'action'], 'Pilot_Result'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.status = kwargs.get('status', int())
        from cola2_msgs.action._pilot import Pilot_Result
        self.result = kwargs.get('result', Pilot_Result())

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
        if self.status != other.status:
            return False
        if self.result != other.result:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'status' field must be an integer in [-128, 127]"
        self._status = value

    @builtins.property
    def result(self):
        """Message field 'result'."""
        return self._result

    @result.setter
    def result(self, value):
        if __debug__:
            from cola2_msgs.action._pilot import Pilot_Result
            assert \
                isinstance(value, Pilot_Result), \
                "The 'result' field must be a sub message of type 'Pilot_Result'"
        self._result = value


class Metaclass_Pilot_GetResult(type):
    """Metaclass of service 'Pilot_GetResult'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('cola2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'cola2_msgs.action.Pilot_GetResult')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__pilot__get_result

            from cola2_msgs.action import _pilot
            if _pilot.Metaclass_Pilot_GetResult_Request._TYPE_SUPPORT is None:
                _pilot.Metaclass_Pilot_GetResult_Request.__import_type_support__()
            if _pilot.Metaclass_Pilot_GetResult_Response._TYPE_SUPPORT is None:
                _pilot.Metaclass_Pilot_GetResult_Response.__import_type_support__()


class Pilot_GetResult(metaclass=Metaclass_Pilot_GetResult):
    from cola2_msgs.action._pilot import Pilot_GetResult_Request as Request
    from cola2_msgs.action._pilot import Pilot_GetResult_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Pilot_FeedbackMessage(type):
    """Metaclass of message 'Pilot_FeedbackMessage'."""

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
                'cola2_msgs.action.Pilot_FeedbackMessage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__pilot__feedback_message
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__pilot__feedback_message
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__pilot__feedback_message
            cls._TYPE_SUPPORT = module.type_support_msg__action__pilot__feedback_message
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__pilot__feedback_message

            from cola2_msgs.action import Pilot
            if Pilot.Feedback.__class__._TYPE_SUPPORT is None:
                Pilot.Feedback.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Pilot_FeedbackMessage(metaclass=Metaclass_Pilot_FeedbackMessage):
    """Message class 'Pilot_FeedbackMessage'."""

    __slots__ = [
        '_goal_id',
        '_feedback',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'feedback': 'cola2_msgs/Pilot_Feedback',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['cola2_msgs', 'action'], 'Pilot_Feedback'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from cola2_msgs.action._pilot import Pilot_Feedback
        self.feedback = kwargs.get('feedback', Pilot_Feedback())

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
        if self.goal_id != other.goal_id:
            return False
        if self.feedback != other.feedback:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def feedback(self):
        """Message field 'feedback'."""
        return self._feedback

    @feedback.setter
    def feedback(self, value):
        if __debug__:
            from cola2_msgs.action._pilot import Pilot_Feedback
            assert \
                isinstance(value, Pilot_Feedback), \
                "The 'feedback' field must be a sub message of type 'Pilot_Feedback'"
        self._feedback = value


class Metaclass_Pilot(type):
    """Metaclass of action 'Pilot'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('cola2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'cola2_msgs.action.Pilot')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_action__action__pilot

            from action_msgs.msg import _goal_status_array
            if _goal_status_array.Metaclass_GoalStatusArray._TYPE_SUPPORT is None:
                _goal_status_array.Metaclass_GoalStatusArray.__import_type_support__()
            from action_msgs.srv import _cancel_goal
            if _cancel_goal.Metaclass_CancelGoal._TYPE_SUPPORT is None:
                _cancel_goal.Metaclass_CancelGoal.__import_type_support__()

            from cola2_msgs.action import _pilot
            if _pilot.Metaclass_Pilot_SendGoal._TYPE_SUPPORT is None:
                _pilot.Metaclass_Pilot_SendGoal.__import_type_support__()
            if _pilot.Metaclass_Pilot_GetResult._TYPE_SUPPORT is None:
                _pilot.Metaclass_Pilot_GetResult.__import_type_support__()
            if _pilot.Metaclass_Pilot_FeedbackMessage._TYPE_SUPPORT is None:
                _pilot.Metaclass_Pilot_FeedbackMessage.__import_type_support__()


class Pilot(metaclass=Metaclass_Pilot):

    # The goal message defined in the action definition.
    from cola2_msgs.action._pilot import Pilot_Goal as Goal
    # The result message defined in the action definition.
    from cola2_msgs.action._pilot import Pilot_Result as Result
    # The feedback message defined in the action definition.
    from cola2_msgs.action._pilot import Pilot_Feedback as Feedback

    class Impl:

        # The send_goal service using a wrapped version of the goal message as a request.
        from cola2_msgs.action._pilot import Pilot_SendGoal as SendGoalService
        # The get_result service using a wrapped version of the result message as a response.
        from cola2_msgs.action._pilot import Pilot_GetResult as GetResultService
        # The feedback message with generic fields which wraps the feedback message.
        from cola2_msgs.action._pilot import Pilot_FeedbackMessage as FeedbackMessage

        # The generic service to cancel a goal.
        from action_msgs.srv._cancel_goal import CancelGoal as CancelGoalService
        # The generic message for get the status of a goal.
        from action_msgs.msg._goal_status_array import GoalStatusArray as GoalStatusMessage

    def __init__(self):
        raise NotImplementedError('Action classes can not be instantiated')
