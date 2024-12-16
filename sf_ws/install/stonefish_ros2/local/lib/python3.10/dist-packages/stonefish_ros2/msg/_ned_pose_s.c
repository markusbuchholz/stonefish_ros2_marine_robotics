// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from stonefish_ros2:msg/NEDPose.idl
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
#include "stonefish_ros2/msg/detail/ned_pose__struct.h"
#include "stonefish_ros2/msg/detail/ned_pose__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool stonefish_ros2__msg__ned_pose__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[37];
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
    assert(strncmp("stonefish_ros2.msg._ned_pose.NEDPose", full_classname_dest, 36) == 0);
  }
  stonefish_ros2__msg__NEDPose * ros_message = _ros_message;
  {  // north
    PyObject * field = PyObject_GetAttrString(_pymsg, "north");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->north = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // east
    PyObject * field = PyObject_GetAttrString(_pymsg, "east");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->east = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // down
    PyObject * field = PyObject_GetAttrString(_pymsg, "down");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->down = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // roll
    PyObject * field = PyObject_GetAttrString(_pymsg, "roll");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->roll = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pitch
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pitch = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yaw = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * stonefish_ros2__msg__ned_pose__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of NEDPose */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("stonefish_ros2.msg._ned_pose");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "NEDPose");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  stonefish_ros2__msg__NEDPose * ros_message = (stonefish_ros2__msg__NEDPose *)raw_ros_message;
  {  // north
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->north);
    {
      int rc = PyObject_SetAttrString(_pymessage, "north", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // east
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->east);
    {
      int rc = PyObject_SetAttrString(_pymessage, "east", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // down
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->down);
    {
      int rc = PyObject_SetAttrString(_pymessage, "down", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // roll
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->roll);
    {
      int rc = PyObject_SetAttrString(_pymessage, "roll", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pitch
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pitch);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pitch", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
