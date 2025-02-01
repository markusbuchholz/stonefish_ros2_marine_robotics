# generated from rosidl_generator_py/resource/_idl.py.em
# with input from stonefish_ros2:srv/SonarSettings.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SonarSettings_Request(type):
    """Metaclass of message 'SonarSettings_Request'."""

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
            module = import_type_support('stonefish_ros2')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'stonefish_ros2.srv.SonarSettings_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__sonar_settings__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__sonar_settings__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__sonar_settings__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__sonar_settings__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__sonar_settings__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SonarSettings_Request(metaclass=Metaclass_SonarSettings_Request):
    """Message class 'SonarSettings_Request'."""

    __slots__ = [
        '_range_min',
        '_range_max',
        '_gain',
    ]

    _fields_and_field_types = {
        'range_min': 'double',
        'range_max': 'double',
        'gain': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.range_min = kwargs.get('range_min', float())
        self.range_max = kwargs.get('range_max', float())
        self.gain = kwargs.get('gain', float())

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
        if self.range_min != other.range_min:
            return False
        if self.range_max != other.range_max:
            return False
        if self.gain != other.gain:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def range_min(self):
        """Message field 'range_min'."""
        return self._range_min

    @range_min.setter
    def range_min(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'range_min' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'range_min' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._range_min = value

    @builtins.property
    def range_max(self):
        """Message field 'range_max'."""
        return self._range_max

    @range_max.setter
    def range_max(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'range_max' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'range_max' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._range_max = value

    @builtins.property
    def gain(self):
        """Message field 'gain'."""
        return self._gain

    @gain.setter
    def gain(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'gain' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'gain' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._gain = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SonarSettings_Response(type):
    """Metaclass of message 'SonarSettings_Response'."""

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
            module = import_type_support('stonefish_ros2')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'stonefish_ros2.srv.SonarSettings_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__sonar_settings__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__sonar_settings__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__sonar_settings__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__sonar_settings__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__sonar_settings__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SonarSettings_Response(metaclass=Metaclass_SonarSettings_Response):
    """Message class 'SonarSettings_Response'."""

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


class Metaclass_SonarSettings(type):
    """Metaclass of service 'SonarSettings'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('stonefish_ros2')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'stonefish_ros2.srv.SonarSettings')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__sonar_settings

            from stonefish_ros2.srv import _sonar_settings
            if _sonar_settings.Metaclass_SonarSettings_Request._TYPE_SUPPORT is None:
                _sonar_settings.Metaclass_SonarSettings_Request.__import_type_support__()
            if _sonar_settings.Metaclass_SonarSettings_Response._TYPE_SUPPORT is None:
                _sonar_settings.Metaclass_SonarSettings_Response.__import_type_support__()


class SonarSettings(metaclass=Metaclass_SonarSettings):
    from stonefish_ros2.srv._sonar_settings import SonarSettings_Request as Request
    from stonefish_ros2.srv._sonar_settings import SonarSettings_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
