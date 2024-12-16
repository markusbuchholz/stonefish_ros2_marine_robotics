# generated from rosidl_generator_py/resource/_idl.py.em
# with input from cola2_msgs:msg/RecoveryAction.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RecoveryAction(type):
    """Metaclass of message 'RecoveryAction'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'NONE': 0,
        'INFORMATIVE': 1,
        'ABORT': 2,
        'ABORT_AND_SURFACE': 3,
        'EMERGENCY_SURFACE': 4,
        'DROP_WEIGHT': 5,
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
                'cola2_msgs.msg.RecoveryAction')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__recovery_action
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__recovery_action
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__recovery_action
            cls._TYPE_SUPPORT = module.type_support_msg__msg__recovery_action
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__recovery_action

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'NONE': cls.__constants['NONE'],
            'INFORMATIVE': cls.__constants['INFORMATIVE'],
            'ABORT': cls.__constants['ABORT'],
            'ABORT_AND_SURFACE': cls.__constants['ABORT_AND_SURFACE'],
            'EMERGENCY_SURFACE': cls.__constants['EMERGENCY_SURFACE'],
            'DROP_WEIGHT': cls.__constants['DROP_WEIGHT'],
        }

    @property
    def NONE(self):
        """Message constant 'NONE'."""
        return Metaclass_RecoveryAction.__constants['NONE']

    @property
    def INFORMATIVE(self):
        """Message constant 'INFORMATIVE'."""
        return Metaclass_RecoveryAction.__constants['INFORMATIVE']

    @property
    def ABORT(self):
        """Message constant 'ABORT'."""
        return Metaclass_RecoveryAction.__constants['ABORT']

    @property
    def ABORT_AND_SURFACE(self):
        """Message constant 'ABORT_AND_SURFACE'."""
        return Metaclass_RecoveryAction.__constants['ABORT_AND_SURFACE']

    @property
    def EMERGENCY_SURFACE(self):
        """Message constant 'EMERGENCY_SURFACE'."""
        return Metaclass_RecoveryAction.__constants['EMERGENCY_SURFACE']

    @property
    def DROP_WEIGHT(self):
        """Message constant 'DROP_WEIGHT'."""
        return Metaclass_RecoveryAction.__constants['DROP_WEIGHT']


class RecoveryAction(metaclass=Metaclass_RecoveryAction):
    """
    Message class 'RecoveryAction'.

    Constants:
      NONE
      INFORMATIVE
      ABORT
      ABORT_AND_SURFACE
      EMERGENCY_SURFACE
      DROP_WEIGHT
    """

    __slots__ = [
        '_header',
        '_error_level',
        '_error_string',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'error_level': 'uint16',
        'error_string': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.error_level = kwargs.get('error_level', int())
        self.error_string = kwargs.get('error_string', str())

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
        if self.error_level != other.error_level:
            return False
        if self.error_string != other.error_string:
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
    def error_level(self):
        """Message field 'error_level'."""
        return self._error_level

    @error_level.setter
    def error_level(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'error_level' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'error_level' field must be an unsigned integer in [0, 65535]"
        self._error_level = value

    @builtins.property
    def error_string(self):
        """Message field 'error_string'."""
        return self._error_string

    @error_string.setter
    def error_string(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'error_string' field must be of type 'str'"
        self._error_string = value
