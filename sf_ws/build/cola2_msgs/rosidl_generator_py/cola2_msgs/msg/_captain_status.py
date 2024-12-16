# generated from rosidl_generator_py/resource/_idl.py.em
# with input from cola2_msgs:msg/CaptainStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CaptainStatus(type):
    """Metaclass of message 'CaptainStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'IDLE': 0,
        'GOTO': 1,
        'MISSION': 2,
        'KEEPPOSITION': 3,
        'SAFETYKEEPPOSITION': 4,
        'EXTERNALMISSION': 5,
        'SECTION': 6,
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
                'cola2_msgs.msg.CaptainStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__captain_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__captain_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__captain_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__captain_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__captain_status

            from cola2_msgs.msg import MissionState
            if MissionState.__class__._TYPE_SUPPORT is None:
                MissionState.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'IDLE': cls.__constants['IDLE'],
            'GOTO': cls.__constants['GOTO'],
            'MISSION': cls.__constants['MISSION'],
            'KEEPPOSITION': cls.__constants['KEEPPOSITION'],
            'SAFETYKEEPPOSITION': cls.__constants['SAFETYKEEPPOSITION'],
            'EXTERNALMISSION': cls.__constants['EXTERNALMISSION'],
            'SECTION': cls.__constants['SECTION'],
        }

    @property
    def IDLE(self):
        """Message constant 'IDLE'."""
        return Metaclass_CaptainStatus.__constants['IDLE']

    @property
    def GOTO(self):
        """Message constant 'GOTO'."""
        return Metaclass_CaptainStatus.__constants['GOTO']

    @property
    def MISSION(self):
        """Message constant 'MISSION'."""
        return Metaclass_CaptainStatus.__constants['MISSION']

    @property
    def KEEPPOSITION(self):
        """Message constant 'KEEPPOSITION'."""
        return Metaclass_CaptainStatus.__constants['KEEPPOSITION']

    @property
    def SAFETYKEEPPOSITION(self):
        """Message constant 'SAFETYKEEPPOSITION'."""
        return Metaclass_CaptainStatus.__constants['SAFETYKEEPPOSITION']

    @property
    def EXTERNALMISSION(self):
        """Message constant 'EXTERNALMISSION'."""
        return Metaclass_CaptainStatus.__constants['EXTERNALMISSION']

    @property
    def SECTION(self):
        """Message constant 'SECTION'."""
        return Metaclass_CaptainStatus.__constants['SECTION']


class CaptainStatus(metaclass=Metaclass_CaptainStatus):
    """
    Message class 'CaptainStatus'.

    Constants:
      IDLE
      GOTO
      MISSION
      KEEPPOSITION
      SAFETYKEEPPOSITION
      EXTERNALMISSION
      SECTION
    """

    __slots__ = [
        '_header',
        '_state',
        '_message',
        '_loaded_missions',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'state': 'uint8',
        'message': 'string',
        'loaded_missions': 'sequence<cola2_msgs/MissionState>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['cola2_msgs', 'msg'], 'MissionState')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.state = kwargs.get('state', int())
        self.message = kwargs.get('message', str())
        self.loaded_missions = kwargs.get('loaded_missions', [])

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
        if self.state != other.state:
            return False
        if self.message != other.message:
            return False
        if self.loaded_missions != other.loaded_missions:
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
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'state' field must be an unsigned integer in [0, 255]"
        self._state = value

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

    @builtins.property
    def loaded_missions(self):
        """Message field 'loaded_missions'."""
        return self._loaded_missions

    @loaded_missions.setter
    def loaded_missions(self, value):
        if __debug__:
            from cola2_msgs.msg import MissionState
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
                 all(isinstance(v, MissionState) for v in value) and
                 True), \
                "The 'loaded_missions' field must be a set or sequence and each value of type 'MissionState'"
        self._loaded_missions = value
