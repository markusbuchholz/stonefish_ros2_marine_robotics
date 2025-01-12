// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from cola2_msgs:msg/NavSts.idl
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
#include "cola2_msgs/msg/detail/nav_sts__struct.h"
#include "cola2_msgs/msg/detail/nav_sts__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
bool cola2_msgs__msg__decimal_lat_lon__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * cola2_msgs__msg__decimal_lat_lon__convert_to_py(void * raw_ros_message);
bool cola2_msgs__msg__decimal_lat_lon__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * cola2_msgs__msg__decimal_lat_lon__convert_to_py(void * raw_ros_message);
bool cola2_msgs__msg__ned__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * cola2_msgs__msg__ned__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__point__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__point__convert_to_py(void * raw_ros_message);
bool cola2_msgs__msg__rpy__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * cola2_msgs__msg__rpy__convert_to_py(void * raw_ros_message);
bool cola2_msgs__msg__rpy__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * cola2_msgs__msg__rpy__convert_to_py(void * raw_ros_message);
bool cola2_msgs__msg__ned__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * cola2_msgs__msg__ned__convert_to_py(void * raw_ros_message);
bool cola2_msgs__msg__rpy__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * cola2_msgs__msg__rpy__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool cola2_msgs__msg__nav_sts__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[31];
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
    assert(strncmp("cola2_msgs.msg._nav_sts.NavSts", full_classname_dest, 30) == 0);
  }
  cola2_msgs__msg__NavSts * ros_message = _ros_message;
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
  {  // global_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "global_position");
    if (!field) {
      return false;
    }
    if (!cola2_msgs__msg__decimal_lat_lon__convert_from_py(field, &ros_message->global_position)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // origin
    PyObject * field = PyObject_GetAttrString(_pymsg, "origin");
    if (!field) {
      return false;
    }
    if (!cola2_msgs__msg__decimal_lat_lon__convert_from_py(field, &ros_message->origin)) {
      Py_DECREF(field);
      return false;
    }
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
  {  // body_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "body_velocity");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__point__convert_from_py(field, &ros_message->body_velocity)) {
      Py_DECREF(field);
      return false;
    }
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
  {  // orientation_rate
    PyObject * field = PyObject_GetAttrString(_pymsg, "orientation_rate");
    if (!field) {
      return false;
    }
    if (!cola2_msgs__msg__rpy__convert_from_py(field, &ros_message->orientation_rate)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // position_variance
    PyObject * field = PyObject_GetAttrString(_pymsg, "position_variance");
    if (!field) {
      return false;
    }
    if (!cola2_msgs__msg__ned__convert_from_py(field, &ros_message->position_variance)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // orientation_variance
    PyObject * field = PyObject_GetAttrString(_pymsg, "orientation_variance");
    if (!field) {
      return false;
    }
    if (!cola2_msgs__msg__rpy__convert_from_py(field, &ros_message->orientation_variance)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * cola2_msgs__msg__nav_sts__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of NavSts */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("cola2_msgs.msg._nav_sts");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "NavSts");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  cola2_msgs__msg__NavSts * ros_message = (cola2_msgs__msg__NavSts *)raw_ros_message;
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
  {  // global_position
    PyObject * field = NULL;
    field = cola2_msgs__msg__decimal_lat_lon__convert_to_py(&ros_message->global_position);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "global_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // origin
    PyObject * field = NULL;
    field = cola2_msgs__msg__decimal_lat_lon__convert_to_py(&ros_message->origin);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "origin", field);
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
  {  // body_velocity
    PyObject * field = NULL;
    field = geometry_msgs__msg__point__convert_to_py(&ros_message->body_velocity);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "body_velocity", field);
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
  {  // orientation_rate
    PyObject * field = NULL;
    field = cola2_msgs__msg__rpy__convert_to_py(&ros_message->orientation_rate);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "orientation_rate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // position_variance
    PyObject * field = NULL;
    field = cola2_msgs__msg__ned__convert_to_py(&ros_message->position_variance);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "position_variance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // orientation_variance
    PyObject * field = NULL;
    field = cola2_msgs__msg__rpy__convert_to_py(&ros_message->orientation_variance);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "orientation_variance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
