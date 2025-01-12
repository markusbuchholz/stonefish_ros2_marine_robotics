# generated from rosidl_generator_py/resource/_idl.py.em
# with input from cola2_msgs:msg/GoalDescriptor.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GoalDescriptor(type):
    """Metaclass of message 'GoalDescriptor'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'PRIORITY_TELEOPERATION_LOW': 0,
        'PRIORITY_SAFETY_LOW': 5,
        'PRIORITY_NORMAL': 10,
        'PRIORITY_SAFETY': 30,
        'PRIORITY_TELEOPERATION': 40,
        'PRIORITY_SAFETY_HIGH': 50,
        'PRIORITY_TELEOPERATION_HIGH': 60,
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
                'cola2_msgs.msg.GoalDescriptor')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__goal_descriptor
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__goal_descriptor
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__goal_descriptor
            cls._TYPE_SUPPORT = module.type_support_msg__msg__goal_descriptor
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__goal_descriptor

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'PRIORITY_TELEOPERATION_LOW': cls.__constants['PRIORITY_TELEOPERATION_LOW'],
            'PRIORITY_SAFETY_LOW': cls.__constants['PRIORITY_SAFETY_LOW'],
            'PRIORITY_NORMAL': cls.__constants['PRIORITY_NORMAL'],
            'PRIORITY_SAFETY': cls.__constants['PRIORITY_SAFETY'],
            'PRIORITY_TELEOPERATION': cls.__constants['PRIORITY_TELEOPERATION'],
            'PRIORITY_SAFETY_HIGH': cls.__constants['PRIORITY_SAFETY_HIGH'],
            'PRIORITY_TELEOPERATION_HIGH': cls.__constants['PRIORITY_TELEOPERATION_HIGH'],
        }

    @property
    def PRIORITY_TELEOPERATION_LOW(self):
        """Message constant 'PRIORITY_TELEOPERATION_LOW'."""
        return Metaclass_GoalDescriptor.__constants['PRIORITY_TELEOPERATION_LOW']

    @property
    def PRIORITY_SAFETY_LOW(self):
        """Message constant 'PRIORITY_SAFETY_LOW'."""
        return Metaclass_GoalDescriptor.__constants['PRIORITY_SAFETY_LOW']

    @property
    def PRIORITY_NORMAL(self):
        """Message constant 'PRIORITY_NORMAL'."""
        return Metaclass_GoalDescriptor.__constants['PRIORITY_NORMAL']

    @property
    def PRIORITY_SAFETY(self):
        """Message constant 'PRIORITY_SAFETY'."""
        return Metaclass_GoalDescriptor.__constants['PRIORITY_SAFETY']

    @property
    def PRIORITY_TELEOPERATION(self):
        """Message constant 'PRIORITY_TELEOPERATION'."""
        return Metaclass_GoalDescriptor.__constants['PRIORITY_TELEOPERATION']

    @property
    def PRIORITY_SAFETY_HIGH(self):
        """Message constant 'PRIORITY_SAFETY_HIGH'."""
        return Metaclass_GoalDescriptor.__constants['PRIORITY_SAFETY_HIGH']

    @property
    def PRIORITY_TELEOPERATION_HIGH(self):
        """Message constant 'PRIORITY_TELEOPERATION_HIGH'."""
        return Metaclass_GoalDescriptor.__constants['PRIORITY_TELEOPERATION_HIGH']


class GoalDescriptor(metaclass=Metaclass_GoalDescriptor):
    """
    Message class 'GoalDescriptor'.

    Constants:
      PRIORITY_TELEOPERATION_LOW
      PRIORITY_SAFETY_LOW
      PRIORITY_NORMAL
      PRIORITY_SAFETY
      PRIORITY_TELEOPERATION
      PRIORITY_SAFETY_HIGH
      PRIORITY_TELEOPERATION_HIGH
    """

    __slots__ = [
        '_requester',
        '_priority',
    ]

    _fields_and_field_types = {
        'requester': 'string',
        'priority': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.requester = kwargs.get('requester', str())
        self.priority = kwargs.get('priority', int())

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
        if self.requester != other.requester:
            return False
        if self.priority != other.priority:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def requester(self):
        """Message field 'requester'."""
        return self._requester

    @requester.setter
    def requester(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'requester' field must be of type 'str'"
        self._requester = value

    @builtins.property
    def priority(self):
        """Message field 'priority'."""
        return self._priority

    @priority.setter
    def priority(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'priority' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'priority' field must be an unsigned integer in [0, 4294967295]"
        self._priority = value
