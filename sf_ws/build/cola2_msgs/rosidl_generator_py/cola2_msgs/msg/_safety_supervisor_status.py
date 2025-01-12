# generated from rosidl_generator_py/resource/_idl.py.em
# with input from cola2_msgs:msg/SafetySupervisorStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SafetySupervisorStatus(type):
    """Metaclass of message 'SafetySupervisorStatus'."""

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
                'cola2_msgs.msg.SafetySupervisorStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__safety_supervisor_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__safety_supervisor_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__safety_supervisor_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__safety_supervisor_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__safety_supervisor_status

            from cola2_msgs.msg import RecoveryAction
            if RecoveryAction.__class__._TYPE_SUPPORT is None:
                RecoveryAction.__class__.__import_type_support__()

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


class SafetySupervisorStatus(metaclass=Metaclass_SafetySupervisorStatus):
    """Message class 'SafetySupervisorStatus'."""

    __slots__ = [
        '_header',
        '_status_code',
        '_recovery_action',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'status_code': 'uint32',
        'recovery_action': 'cola2_msgs/RecoveryAction',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['cola2_msgs', 'msg'], 'RecoveryAction'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.status_code = kwargs.get('status_code', int())
        from cola2_msgs.msg import RecoveryAction
        self.recovery_action = kwargs.get('recovery_action', RecoveryAction())

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
        if self.status_code != other.status_code:
            return False
        if self.recovery_action != other.recovery_action:
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
    def status_code(self):
        """Message field 'status_code'."""
        return self._status_code

    @status_code.setter
    def status_code(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status_code' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'status_code' field must be an unsigned integer in [0, 4294967295]"
        self._status_code = value

    @builtins.property
    def recovery_action(self):
        """Message field 'recovery_action'."""
        return self._recovery_action

    @recovery_action.setter
    def recovery_action(self, value):
        if __debug__:
            from cola2_msgs.msg import RecoveryAction
            assert \
                isinstance(value, RecoveryAction), \
                "The 'recovery_action' field must be a sub message of type 'RecoveryAction'"
        self._recovery_action = value
