// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from stonefish_ros2:msg/DebugPhysics.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "stonefish_ros2/msg/detail/debug_physics__struct.h"
#include "stonefish_ros2/msg/detail/debug_physics__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__vector3__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__vector3__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__vector3__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__vector3__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__vector3__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__vector3__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__vector3__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__vector3__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__vector3__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__vector3__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__twist__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__twist__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__wrench__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__wrench__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__wrench__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__wrench__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__wrench__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__wrench__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool stonefish_ros2__msg__debug_physics__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[47];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("stonefish_ros2.msg._debug_physics.DebugPhysics", full_classname_dest, 46) == 0);
  }
  stonefish_ros2__msg__DebugPhysics * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // mass
    PyObject * field = PyObject_GetAttrString(_pymsg, "mass");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->mass = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // volume
    PyObject * field = PyObject_GetAttrString(_pymsg, "volume");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->volume = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // surface
    PyObject * field = PyObject_GetAttrString(_pymsg, "surface");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->surface = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cog
    PyObject * field = PyObject_GetAttrString(_pymsg, "cog");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__vector3__convert_from_py(field, &ros_message->cog)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // cob
    PyObject * field = PyObject_GetAttrString(_pymsg, "cob");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__vector3__convert_from_py(field, &ros_message->cob)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // inertia
    PyObject * field = PyObject_GetAttrString(_pymsg, "inertia");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__vector3__convert_from_py(field, &ros_message->inertia)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // damping_coeff
    PyObject * field = PyObject_GetAttrString(_pymsg, "damping_coeff");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__vector3__convert_from_py(field, &ros_message->damping_coeff)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // skin_friction_coeff
    PyObject * field = PyObject_GetAttrString(_pymsg, "skin_friction_coeff");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__vector3__convert_from_py(field, &ros_message->skin_friction_coeff)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "velocity");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__twist__convert_from_py(field, &ros_message->velocity)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // buoyancy
    PyObject * field = PyObject_GetAttrString(_pymsg, "buoyancy");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__wrench__convert_from_py(field, &ros_message->buoyancy)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // damping
    PyObject * field = PyObject_GetAttrString(_pymsg, "damping");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__wrench__convert_from_py(field, &ros_message->damping)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // skin_friction
    PyObject * field = PyObject_GetAttrString(_pymsg, "skin_friction");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__wrench__convert_from_py(field, &ros_message->skin_friction)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // wetted_surface
    PyObject * field = PyObject_GetAttrString(_pymsg, "wetted_surface");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->wetted_surface = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // submerged_volume
    PyObject * field = PyObject_GetAttrString(_pymsg, "submerged_volume");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->submerged_volume = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * stonefish_ros2__msg__debug_physics__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of DebugPhysics */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("stonefish_ros2.msg._debug_physics");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "DebugPhysics");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  stonefish_ros2__msg__DebugPhysics * ros_message = (stonefish_ros2__msg__DebugPhysics *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // mass
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->mass);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mass", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // volume
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->volume);
    {
      int rc = PyObject_SetAttrString(_pymessage, "volume", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // surface
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->surface);
    {
      int rc = PyObject_SetAttrString(_pymessage, "surface", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cog
    PyObject * field = NULL;
    field = geometry_msgs__msg__vector3__convert_to_py(&ros_message->cog);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "cog", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cob
    PyObject * field = NULL;
    field = geometry_msgs__msg__vector3__convert_to_py(&ros_message->cob);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "cob", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // inertia
    PyObject * field = NULL;
    field = geometry_msgs__msg__vector3__convert_to_py(&ros_message->inertia);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "inertia", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // damping_coeff
    PyObject * field = NULL;
    field = geometry_msgs__msg__vector3__convert_to_py(&ros_message->damping_coeff);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "damping_coeff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // skin_friction_coeff
    PyObject * field = NULL;
    field = geometry_msgs__msg__vector3__convert_to_py(&ros_message->skin_friction_coeff);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "skin_friction_coeff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // velocity
    PyObject * field = NULL;
    field = geometry_msgs__msg__twist__convert_to_py(&ros_message->velocity);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // buoyancy
    PyObject * field = NULL;
    field = geometry_msgs__msg__wrench__convert_to_py(&ros_message->buoyancy);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "buoyancy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // damping
    PyObject * field = NULL;
    field = geometry_msgs__msg__wrench__convert_to_py(&ros_message->damping);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "damping", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // skin_friction
    PyObject * field = NULL;
    field = geometry_msgs__msg__wrench__convert_to_py(&ros_message->skin_friction);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "skin_friction", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // wetted_surface
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->wetted_surface);
    {
      int rc = PyObject_SetAttrString(_pymessage, "wetted_surface", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // submerged_volume
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->submerged_volume);
    {
      int rc = PyObject_SetAttrString(_pymessage, "submerged_volume", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
