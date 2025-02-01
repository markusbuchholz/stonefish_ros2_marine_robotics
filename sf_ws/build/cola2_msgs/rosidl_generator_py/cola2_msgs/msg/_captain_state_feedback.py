# generated from rosidl_generator_py/resource/_idl.py.em
# with input from cola2_msgs:msg/CaptainStateFeedback.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CaptainStateFeedback(type):
    """Metaclass of message 'CaptainStateFeedback'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'ACTIVE': 0,
        'SUCCESS': 1,
        'FAILURE': 2,
        'STOPPED': 3,
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
                'cola2_msgs.msg.CaptainStateFeedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__captain_state_feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__captain_state_feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__captain_state_feedback
            cls._TYPE_SUPPORT = module.type_support_msg__msg__captain_state_feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__captain_state_feedback

            from cola2_msgs.msg import KeyValue
            if KeyValue.__class__._TYPE_SUPPORT is None:
                KeyValue.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'ACTIVE': cls.__constants['ACTIVE'],
            'SUCCESS': cls.__constants['SUCCESS'],
            'FAILURE': cls.__constants['FAILURE'],
            'STOPPED': cls.__constants['STOPPED'],
        }

    @property
    def ACTIVE(self):
        """Message constant 'ACTIVE'."""
        return Metaclass_CaptainStateFeedback.__constants['ACTIVE']

    @property
    def SUCCESS(self):
        """Message constant 'SUCCESS'."""
        return Metaclass_CaptainStateFeedback.__constants['SUCCESS']

    @property
    def FAILURE(self):
        """Message constant 'FAILURE'."""
        return Metaclass_CaptainStateFeedback.__constants['FAILURE']

    @property
    def STOPPED(self):
        """Message constant 'STOPPED'."""
        return Metaclass_CaptainStateFeedback.__constants['STOPPED']


class CaptainStateFeedback(metaclass=Metaclass_CaptainStateFeedback):
    """
    Message class 'CaptainStateFeedback'.

    Constants:
      ACTIVE
      SUCCESS
      FAILURE
      STOPPED
    """

    __slots__ = [
        '_header',
        '_state',
        '_time_to_finish',
        '_name',
        '_id',
        '_keyvalues',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'state': 'uint8',
        'time_to_finish': 'double',
        'name': 'string',
        'id': 'uint16',
        'keyvalues': 'sequence<cola2_msgs/KeyValue>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['cola2_msgs', 'msg'], 'KeyValue')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.state = kwargs.get('state', int())
        self.time_to_finish = kwargs.get('time_to_finish', float())
        self.name = kwargs.get('name', str())
        self.id = kwargs.get('id', int())
        self.keyvalues = kwargs.get('keyvalues', [])

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
        if self.time_to_finish != other.time_to_finish:
            return False
        if self.name != other.name:
            return False
        if self.id != other.id:
            return False
        if self.keyvalues != other.keyvalues:
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
    def time_to_finish(self):
        """Message field 'time_to_finish'."""
        return self._time_to_finish

    @time_to_finish.setter
    def time_to_finish(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'time_to_finish' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'time_to_finish' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._time_to_finish = value

    @builtins.property
    def name(self):
        """Message field 'name'."""
        return self._name

    @name.setter
    def name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'name' field must be of type 'str'"
        self._name = value

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'id' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'id' field must be an unsigned integer in [0, 65535]"
        self._id = value

    @builtins.property
    def keyvalues(self):
        """Message field 'keyvalues'."""
        return self._keyvalues

    @keyvalues.setter
    def keyvalues(self, value):
        if __debug__:
            from cola2_msgs.msg import KeyValue
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
                 all(isinstance(v, KeyValue) for v in value) and
                 True), \
                "The 'keyvalues' field must be a set or sequence and each value of type 'KeyValue'"
        self._keyvalues = value
