# generated from rosidl_generator_py/resource/_idl.py.em
# with input from cola2_msgs:msg/BodyVelocityReq.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BodyVelocityReq(type):
    """Metaclass of message 'BodyVelocityReq'."""

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
                'cola2_msgs.msg.BodyVelocityReq')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__body_velocity_req
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__body_velocity_req
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__body_velocity_req
            cls._TYPE_SUPPORT = module.type_support_msg__msg__body_velocity_req
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__body_velocity_req

            from cola2_msgs.msg import Bool6Axis
            if Bool6Axis.__class__._TYPE_SUPPORT is None:
                Bool6Axis.__class__.__import_type_support__()

            from cola2_msgs.msg import GoalDescriptor
            if GoalDescriptor.__class__._TYPE_SUPPORT is None:
                GoalDescriptor.__class__.__import_type_support__()

            from geometry_msgs.msg import Twist
            if Twist.__class__._TYPE_SUPPORT is None:
                Twist.__class__.__import_type_support__()

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


class BodyVelocityReq(metaclass=Metaclass_BodyVelocityReq):
    """Message class 'BodyVelocityReq'."""

    __slots__ = [
        '_header',
        '_goal',
        '_twist',
        '_disable_axis',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'goal': 'cola2_msgs/GoalDescriptor',
        'twist': 'geometry_msgs/Twist',
        'disable_axis': 'cola2_msgs/Bool6Axis',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['cola2_msgs', 'msg'], 'GoalDescriptor'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Twist'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['cola2_msgs', 'msg'], 'Bool6Axis'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from cola2_msgs.msg import GoalDescriptor
        self.goal = kwargs.get('goal', GoalDescriptor())
        from geometry_msgs.msg import Twist
        self.twist = kwargs.get('twist', Twist())
        from cola2_msgs.msg import Bool6Axis
        self.disable_axis = kwargs.get('disable_axis', Bool6Axis())

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
        if self.twist != other.twist:
            return False
        if self.disable_axis != other.disable_axis:
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
    def twist(self):
        """Message field 'twist'."""
        return self._twist

    @twist.setter
    def twist(self, value):
        if __debug__:
            from geometry_msgs.msg import Twist
            assert \
                isinstance(value, Twist), \
                "The 'twist' field must be a sub message of type 'Twist'"
        self._twist = value

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
