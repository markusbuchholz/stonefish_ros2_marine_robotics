# generated from rosidl_generator_py/resource/_idl.py.em
# with input from cola2_msgs:srv/Section.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Section_Request(type):
    """Metaclass of message 'Section_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'NED': 0,
        'GLOBAL': 1,
        'DEPTH': 0,
        'ALTITUDE': 1,
        'BOTH': 2,
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
                'cola2_msgs.srv.Section_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__section__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__section__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__section__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__section__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__section__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'NED': cls.__constants['NED'],
            'GLOBAL': cls.__constants['GLOBAL'],
            'DEPTH': cls.__constants['DEPTH'],
            'ALTITUDE': cls.__constants['ALTITUDE'],
            'BOTH': cls.__constants['BOTH'],
        }

    @property
    def NED(self):
        """Message constant 'NED'."""
        return Metaclass_Section_Request.__constants['NED']

    @property
    def GLOBAL(self):
        """Message constant 'GLOBAL'."""
        return Metaclass_Section_Request.__constants['GLOBAL']

    @property
    def DEPTH(self):
        """Message constant 'DEPTH'."""
        return Metaclass_Section_Request.__constants['DEPTH']

    @property
    def ALTITUDE(self):
        """Message constant 'ALTITUDE'."""
        return Metaclass_Section_Request.__constants['ALTITUDE']

    @property
    def BOTH(self):
        """Message constant 'BOTH'."""
        return Metaclass_Section_Request.__constants['BOTH']


class Section_Request(metaclass=Metaclass_Section_Request):
    """
    Message class 'Section_Request'.

    Constants:
      NED
      GLOBAL
      DEPTH
      ALTITUDE
      BOTH
    """

    __slots__ = [
        '_initial_x',
        '_initial_y',
        '_initial_depth',
        '_final_x',
        '_final_y',
        '_final_depth',
        '_final_altitude',
        '_reference',
        '_heave_mode',
        '_surge_velocity',
        '_tolerance_xy',
        '_timeout',
        '_no_altitude_goes_up',
    ]

    _fields_and_field_types = {
        'initial_x': 'double',
        'initial_y': 'double',
        'initial_depth': 'double',
        'final_x': 'double',
        'final_y': 'double',
        'final_depth': 'double',
        'final_altitude': 'double',
        'reference': 'uint8',
        'heave_mode': 'uint64',
        'surge_velocity': 'double',
        'tolerance_xy': 'double',
        'timeout': 'double',
        'no_altitude_goes_up': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.initial_x = kwargs.get('initial_x', float())
        self.initial_y = kwargs.get('initial_y', float())
        self.initial_depth = kwargs.get('initial_depth', float())
        self.final_x = kwargs.get('final_x', float())
        self.final_y = kwargs.get('final_y', float())
        self.final_depth = kwargs.get('final_depth', float())
        self.final_altitude = kwargs.get('final_altitude', float())
        self.reference = kwargs.get('reference', int())
        self.heave_mode = kwargs.get('heave_mode', int())
        self.surge_velocity = kwargs.get('surge_velocity', float())
        self.tolerance_xy = kwargs.get('tolerance_xy', float())
        self.timeout = kwargs.get('timeout', float())
        self.no_altitude_goes_up = kwargs.get('no_altitude_goes_up', bool())

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
        if self.initial_x != other.initial_x:
            return False
        if self.initial_y != other.initial_y:
            return False
        if self.initial_depth != other.initial_depth:
            return False
        if self.final_x != other.final_x:
            return False
        if self.final_y != other.final_y:
            return False
        if self.final_depth != other.final_depth:
            return False
        if self.final_altitude != other.final_altitude:
            return False
        if self.reference != other.reference:
            return False
        if self.heave_mode != other.heave_mode:
            return False
        if self.surge_velocity != other.surge_velocity:
            return False
        if self.tolerance_xy != other.tolerance_xy:
            return False
        if self.timeout != other.timeout:
            return False
        if self.no_altitude_goes_up != other.no_altitude_goes_up:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def initial_x(self):
        """Message field 'initial_x'."""
        return self._initial_x

    @initial_x.setter
    def initial_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'initial_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'initial_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._initial_x = value

    @builtins.property
    def initial_y(self):
        """Message field 'initial_y'."""
        return self._initial_y

    @initial_y.setter
    def initial_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'initial_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'initial_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._initial_y = value

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
    def final_x(self):
        """Message field 'final_x'."""
        return self._final_x

    @final_x.setter
    def final_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'final_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'final_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._final_x = value

    @builtins.property
    def final_y(self):
        """Message field 'final_y'."""
        return self._final_y

    @final_y.setter
    def final_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'final_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'final_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._final_y = value

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
    def reference(self):
        """Message field 'reference'."""
        return self._reference

    @reference.setter
    def reference(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'reference' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'reference' field must be an unsigned integer in [0, 255]"
        self._reference = value

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
    def no_altitude_goes_up(self):
        """Message field 'no_altitude_goes_up'."""
        return self._no_altitude_goes_up

    @no_altitude_goes_up.setter
    def no_altitude_goes_up(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'no_altitude_goes_up' field must be of type 'bool'"
        self._no_altitude_goes_up = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Section_Response(type):
    """Metaclass of message 'Section_Response'."""

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
                'cola2_msgs.srv.Section_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__section__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__section__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__section__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__section__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__section__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Section_Response(metaclass=Metaclass_Section_Response):
    """Message class 'Section_Response'."""

    __slots__ = [
        '_success',
        '_message',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())

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
        if self.success != other.success:
            return False
        if self.message != other.message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value


class Metaclass_Section(type):
    """Metaclass of service 'Section'."""

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
                'cola2_msgs.srv.Section')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__section

            from cola2_msgs.srv import _section
            if _section.Metaclass_Section_Request._TYPE_SUPPORT is None:
                _section.Metaclass_Section_Request.__import_type_support__()
            if _section.Metaclass_Section_Response._TYPE_SUPPORT is None:
                _section.Metaclass_Section_Response.__import_type_support__()


class Section(metaclass=Metaclass_Section):
    from cola2_msgs.srv._section import Section_Request as Request
    from cola2_msgs.srv._section import Section_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
