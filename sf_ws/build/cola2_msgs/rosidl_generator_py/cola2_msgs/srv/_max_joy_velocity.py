# generated from rosidl_generator_py/resource/_idl.py.em
# with input from cola2_msgs:srv/MaxJoyVelocity.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'max_joy_velocity'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MaxJoyVelocity_Request(type):
    """Metaclass of message 'MaxJoyVelocity_Request'."""

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
                'cola2_msgs.srv.MaxJoyVelocity_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__max_joy_velocity__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__max_joy_velocity__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__max_joy_velocity__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__max_joy_velocity__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__max_joy_velocity__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MaxJoyVelocity_Request(metaclass=Metaclass_MaxJoyVelocity_Request):
    """Message class 'MaxJoyVelocity_Request'."""

    __slots__ = [
        '_max_joy_velocity',
    ]

    _fields_and_field_types = {
        'max_joy_velocity': 'double[6]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 6),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        if 'max_joy_velocity' not in kwargs:
            self.max_joy_velocity = numpy.zeros(6, dtype=numpy.float64)
        else:
            self.max_joy_velocity = numpy.array(kwargs.get('max_joy_velocity'), dtype=numpy.float64)
            assert self.max_joy_velocity.shape == (6, )

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
        if all(self.max_joy_velocity != other.max_joy_velocity):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def max_joy_velocity(self):
        """Message field 'max_joy_velocity'."""
        return self._max_joy_velocity

    @max_joy_velocity.setter
    def max_joy_velocity(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'max_joy_velocity' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 6, \
                "The 'max_joy_velocity' numpy.ndarray() must have a size of 6"
            self._max_joy_velocity = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 6 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'max_joy_velocity' field must be a set or sequence with length 6 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._max_joy_velocity = numpy.array(value, dtype=numpy.float64)


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_MaxJoyVelocity_Response(type):
    """Metaclass of message 'MaxJoyVelocity_Response'."""

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
                'cola2_msgs.srv.MaxJoyVelocity_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__max_joy_velocity__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__max_joy_velocity__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__max_joy_velocity__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__max_joy_velocity__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__max_joy_velocity__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MaxJoyVelocity_Response(metaclass=Metaclass_MaxJoyVelocity_Response):
    """Message class 'MaxJoyVelocity_Response'."""

    __slots__ = [
        '_attempted',
    ]

    _fields_and_field_types = {
        'attempted': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.attempted = kwargs.get('attempted', bool())

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
        if self.attempted != other.attempted:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def attempted(self):
        """Message field 'attempted'."""
        return self._attempted

    @attempted.setter
    def attempted(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'attempted' field must be of type 'bool'"
        self._attempted = value


class Metaclass_MaxJoyVelocity(type):
    """Metaclass of service 'MaxJoyVelocity'."""

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
                'cola2_msgs.srv.MaxJoyVelocity')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__max_joy_velocity

            from cola2_msgs.srv import _max_joy_velocity
            if _max_joy_velocity.Metaclass_MaxJoyVelocity_Request._TYPE_SUPPORT is None:
                _max_joy_velocity.Metaclass_MaxJoyVelocity_Request.__import_type_support__()
            if _max_joy_velocity.Metaclass_MaxJoyVelocity_Response._TYPE_SUPPORT is None:
                _max_joy_velocity.Metaclass_MaxJoyVelocity_Response.__import_type_support__()


class MaxJoyVelocity(metaclass=Metaclass_MaxJoyVelocity):
    from cola2_msgs.srv._max_joy_velocity import MaxJoyVelocity_Request as Request
    from cola2_msgs.srv._max_joy_velocity import MaxJoyVelocity_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
