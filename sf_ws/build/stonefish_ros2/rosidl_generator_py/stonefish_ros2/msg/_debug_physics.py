# generated from rosidl_generator_py/resource/_idl.py.em
# with input from stonefish_ros2:msg/DebugPhysics.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DebugPhysics(type):
    """Metaclass of message 'DebugPhysics'."""

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
                'stonefish_ros2.msg.DebugPhysics')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__debug_physics
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__debug_physics
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__debug_physics
            cls._TYPE_SUPPORT = module.type_support_msg__msg__debug_physics
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__debug_physics

            from geometry_msgs.msg import Twist
            if Twist.__class__._TYPE_SUPPORT is None:
                Twist.__class__.__import_type_support__()

            from geometry_msgs.msg import Vector3
            if Vector3.__class__._TYPE_SUPPORT is None:
                Vector3.__class__.__import_type_support__()

            from geometry_msgs.msg import Wrench
            if Wrench.__class__._TYPE_SUPPORT is None:
                Wrench.__class__.__import_type_support__()

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


class DebugPhysics(metaclass=Metaclass_DebugPhysics):
    """Message class 'DebugPhysics'."""

    __slots__ = [
        '_header',
        '_mass',
        '_volume',
        '_surface',
        '_cog',
        '_cob',
        '_inertia',
        '_damping_coeff',
        '_skin_friction_coeff',
        '_velocity',
        '_buoyancy',
        '_damping',
        '_skin_friction',
        '_wetted_surface',
        '_submerged_volume',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'mass': 'double',
        'volume': 'double',
        'surface': 'double',
        'cog': 'geometry_msgs/Vector3',
        'cob': 'geometry_msgs/Vector3',
        'inertia': 'geometry_msgs/Vector3',
        'damping_coeff': 'geometry_msgs/Vector3',
        'skin_friction_coeff': 'geometry_msgs/Vector3',
        'velocity': 'geometry_msgs/Twist',
        'buoyancy': 'geometry_msgs/Wrench',
        'damping': 'geometry_msgs/Wrench',
        'skin_friction': 'geometry_msgs/Wrench',
        'wetted_surface': 'double',
        'submerged_volume': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Twist'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Wrench'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Wrench'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Wrench'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.mass = kwargs.get('mass', float())
        self.volume = kwargs.get('volume', float())
        self.surface = kwargs.get('surface', float())
        from geometry_msgs.msg import Vector3
        self.cog = kwargs.get('cog', Vector3())
        from geometry_msgs.msg import Vector3
        self.cob = kwargs.get('cob', Vector3())
        from geometry_msgs.msg import Vector3
        self.inertia = kwargs.get('inertia', Vector3())
        from geometry_msgs.msg import Vector3
        self.damping_coeff = kwargs.get('damping_coeff', Vector3())
        from geometry_msgs.msg import Vector3
        self.skin_friction_coeff = kwargs.get('skin_friction_coeff', Vector3())
        from geometry_msgs.msg import Twist
        self.velocity = kwargs.get('velocity', Twist())
        from geometry_msgs.msg import Wrench
        self.buoyancy = kwargs.get('buoyancy', Wrench())
        from geometry_msgs.msg import Wrench
        self.damping = kwargs.get('damping', Wrench())
        from geometry_msgs.msg import Wrench
        self.skin_friction = kwargs.get('skin_friction', Wrench())
        self.wetted_surface = kwargs.get('wetted_surface', float())
        self.submerged_volume = kwargs.get('submerged_volume', float())

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
        if self.mass != other.mass:
            return False
        if self.volume != other.volume:
            return False
        if self.surface != other.surface:
            return False
        if self.cog != other.cog:
            return False
        if self.cob != other.cob:
            return False
        if self.inertia != other.inertia:
            return False
        if self.damping_coeff != other.damping_coeff:
            return False
        if self.skin_friction_coeff != other.skin_friction_coeff:
            return False
        if self.velocity != other.velocity:
            return False
        if self.buoyancy != other.buoyancy:
            return False
        if self.damping != other.damping:
            return False
        if self.skin_friction != other.skin_friction:
            return False
        if self.wetted_surface != other.wetted_surface:
            return False
        if self.submerged_volume != other.submerged_volume:
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
    def mass(self):
        """Message field 'mass'."""
        return self._mass

    @mass.setter
    def mass(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'mass' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'mass' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._mass = value

    @builtins.property
    def volume(self):
        """Message field 'volume'."""
        return self._volume

    @volume.setter
    def volume(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'volume' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'volume' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._volume = value

    @builtins.property
    def surface(self):
        """Message field 'surface'."""
        return self._surface

    @surface.setter
    def surface(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'surface' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'surface' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._surface = value

    @builtins.property
    def cog(self):
        """Message field 'cog'."""
        return self._cog

    @cog.setter
    def cog(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'cog' field must be a sub message of type 'Vector3'"
        self._cog = value

    @builtins.property
    def cob(self):
        """Message field 'cob'."""
        return self._cob

    @cob.setter
    def cob(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'cob' field must be a sub message of type 'Vector3'"
        self._cob = value

    @builtins.property
    def inertia(self):
        """Message field 'inertia'."""
        return self._inertia

    @inertia.setter
    def inertia(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'inertia' field must be a sub message of type 'Vector3'"
        self._inertia = value

    @builtins.property
    def damping_coeff(self):
        """Message field 'damping_coeff'."""
        return self._damping_coeff

    @damping_coeff.setter
    def damping_coeff(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'damping_coeff' field must be a sub message of type 'Vector3'"
        self._damping_coeff = value

    @builtins.property
    def skin_friction_coeff(self):
        """Message field 'skin_friction_coeff'."""
        return self._skin_friction_coeff

    @skin_friction_coeff.setter
    def skin_friction_coeff(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'skin_friction_coeff' field must be a sub message of type 'Vector3'"
        self._skin_friction_coeff = value

    @builtins.property
    def velocity(self):
        """Message field 'velocity'."""
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        if __debug__:
            from geometry_msgs.msg import Twist
            assert \
                isinstance(value, Twist), \
                "The 'velocity' field must be a sub message of type 'Twist'"
        self._velocity = value

    @builtins.property
    def buoyancy(self):
        """Message field 'buoyancy'."""
        return self._buoyancy

    @buoyancy.setter
    def buoyancy(self, value):
        if __debug__:
            from geometry_msgs.msg import Wrench
            assert \
                isinstance(value, Wrench), \
                "The 'buoyancy' field must be a sub message of type 'Wrench'"
        self._buoyancy = value

    @builtins.property
    def damping(self):
        """Message field 'damping'."""
        return self._damping

    @damping.setter
    def damping(self, value):
        if __debug__:
            from geometry_msgs.msg import Wrench
            assert \
                isinstance(value, Wrench), \
                "The 'damping' field must be a sub message of type 'Wrench'"
        self._damping = value

    @builtins.property
    def skin_friction(self):
        """Message field 'skin_friction'."""
        return self._skin_friction

    @skin_friction.setter
    def skin_friction(self, value):
        if __debug__:
            from geometry_msgs.msg import Wrench
            assert \
                isinstance(value, Wrench), \
                "The 'skin_friction' field must be a sub message of type 'Wrench'"
        self._skin_friction = value

    @builtins.property
    def wetted_surface(self):
        """Message field 'wetted_surface'."""
        return self._wetted_surface

    @wetted_surface.setter
    def wetted_surface(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'wetted_surface' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'wetted_surface' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._wetted_surface = value

    @builtins.property
    def submerged_volume(self):
        """Message field 'submerged_volume'."""
        return self._submerged_volume

    @submerged_volume.setter
    def submerged_volume(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'submerged_volume' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'submerged_volume' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._submerged_volume = value
