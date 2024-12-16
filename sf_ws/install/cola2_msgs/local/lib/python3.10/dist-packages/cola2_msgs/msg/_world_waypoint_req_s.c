// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from cola2_msgs:msg/WorldWaypointReq.idl
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
#include "cola2_msgs/msg/detail/world_waypoint_req__struct.h"
#include "cola2_msgs/msg/detail/world_waypoint_req__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
bool cola2_msgs__msg__goal_descriptor__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * cola2_msgs__msg__goal_descriptor__convert_to_py(void * raw_ros_message);
bool cola2_msgs__msg__ned__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * cola2_msgs__msg__ned__convert_to_py(void * raw_ros_message);
bool cola2_msgs__msg__rpy__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * cola2_msgs__msg__rpy__convert_to_py(void * raw_ros_message);
bool cola2_msgs__msg__bool6_axis__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * cola2_msgs__msg__bool6_axis__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__vector3__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__vector3__convert_to_py(void * raw_ros_message);
bool cola2_msgs__msg__rpy__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * cola2_msgs__msg__rpy__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool cola2_msgs__msg__world_waypoint_req__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[52];
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
    assert(strncmp("cola2_msgs.msg._world_waypoint_req.WorldWaypointReq", full_classname_dest, 51) == 0);
  }
  cola2_msgs__msg__WorldWaypointReq * ros_message = _ros_message;
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
  {  // goal
    PyObject * field = PyObject_GetAttrString(_pymsg, "goal");
    if (!field) {
      return false;
    }
    if (!cola2_msgs__msg__goal_descriptor__convert_from_py(field, &ros_message->goal)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // altitude_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "altitude_mode");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->altitude_mode = (Py_True == field);
    Py_DECREF(field);
  }
  {  // position
    PyObject * field = PyObject_GetAttrString(_pymsg, "position");
    if (!field) {
      return false;
    }
    if (!cola2_msgs__msg__ned__convert_from_py(field, &ros_message->position)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // altitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "altitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->altitude = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // orientation
    PyObject * field = PyObject_GetAttrString(_pymsg, "orientation");
    if (!field) {
      return false;
    }
    if (!cola2_msgs__msg__rpy__convert_from_py(field, &ros_message->orientation)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // disable_axis
    PyObject * field = PyObject_GetAttrString(_pymsg, "disable_axis");
    if (!field) {
      return false;
    }
    if (!cola2_msgs__msg__bool6_axis__convert_from_py(field, &ros_message->disable_axis)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // position_tolerance
    PyObject * field = PyObject_GetAttrString(_pymsg, "position_tolerance");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__vector3__convert_from_py(field, &ros_message->position_tolerance)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // orientation_tolerance
    PyObject * field = PyObject_GetAttrString(_pymsg, "orientation_tolerance");
    if (!field) {
      return false;
    }
    if (!cola2_msgs__msg__rpy__convert_from_py(field, &ros_message->orientation_tolerance)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * cola2_msgs__msg__world_waypoint_req__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of WorldWaypointReq */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("cola2_msgs.msg._world_waypoint_req");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "WorldWaypointReq");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  cola2_msgs__msg__WorldWaypointReq * ros_message = (cola2_msgs__msg__WorldWaypointReq *)raw_ros_message;
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
  {  // goal
    PyObject * field = NULL;
    field = cola2_msgs__msg__goal_descriptor__convert_to_py(&ros_message->goal);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "goal", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // altitude_mode
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->altitude_mode ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "altitude_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // position
    PyObject * field = NULL;
    field = cola2_msgs__msg__ned__convert_to_py(&ros_message->position);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // altitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->altitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "altitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // orientation
    PyObject * field = NULL;
    field = cola2_msgs__msg__rpy__convert_to_py(&ros_message->orientation);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "orientation", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // disable_axis
    PyObject * field = NULL;
    field = cola2_msgs__msg__bool6_axis__convert_to_py(&ros_message->disable_axis);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "disable_axis", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // position_tolerance
    PyObject * field = NULL;
    field = geometry_msgs__msg__vector3__convert_to_py(&ros_message->position_tolerance);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "position_tolerance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // orientation_tolerance
    PyObject * field = NULL;
    field = cola2_msgs__msg__rpy__convert_to_py(&ros_message->orientation_tolerance);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "orientation_tolerance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
