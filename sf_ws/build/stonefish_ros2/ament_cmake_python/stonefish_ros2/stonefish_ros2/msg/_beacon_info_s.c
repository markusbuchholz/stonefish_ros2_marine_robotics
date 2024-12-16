// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from stonefish_ros2:msg/BeaconInfo.idl
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
#include "stonefish_ros2/msg/detail/beacon_info__struct.h"
#include "stonefish_ros2/msg/detail/beacon_info__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__point__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__point__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__quaternion__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__quaternion__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool stonefish_ros2__msg__beacon_info__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[43];
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
    assert(strncmp("stonefish_ros2.msg._beacon_info.BeaconInfo", full_classname_dest, 42) == 0);
  }
  stonefish_ros2__msg__BeaconInfo * ros_message = _ros_message;
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
  {  // beacon_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "beacon_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->beacon_id = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // range
    PyObject * field = PyObject_GetAttrString(_pymsg, "range");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->range = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // azimuth
    PyObject * field = PyObject_GetAttrString(_pymsg, "azimuth");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->azimuth = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // elevation
    PyObject * field = PyObject_GetAttrString(_pymsg, "elevation");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->elevation = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // relative_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "relative_position");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__point__convert_from_py(field, &ros_message->relative_position)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // local_orientation
    PyObject * field = PyObject_GetAttrString(_pymsg, "local_orientation");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__quaternion__convert_from_py(field, &ros_message->local_orientation)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // local_depth
    PyObject * field = PyObject_GetAttrString(_pymsg, "local_depth");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->local_depth = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * stonefish_ros2__msg__beacon_info__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of BeaconInfo */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("stonefish_ros2.msg._beacon_info");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "BeaconInfo");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  stonefish_ros2__msg__BeaconInfo * ros_message = (stonefish_ros2__msg__BeaconInfo *)raw_ros_message;
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
  {  // beacon_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->beacon_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "beacon_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // range
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->range);
    {
      int rc = PyObject_SetAttrString(_pymessage, "range", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // azimuth
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->azimuth);
    {
      int rc = PyObject_SetAttrString(_pymessage, "azimuth", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // elevation
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->elevation);
    {
      int rc = PyObject_SetAttrString(_pymessage, "elevation", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // relative_position
    PyObject * field = NULL;
    field = geometry_msgs__msg__point__convert_to_py(&ros_message->relative_position);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "relative_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // local_orientation
    PyObject * field = NULL;
    field = geometry_msgs__msg__quaternion__convert_to_py(&ros_message->local_orientation);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "local_orientation", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // local_depth
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->local_depth);
    {
      int rc = PyObject_SetAttrString(_pymessage, "local_depth", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
