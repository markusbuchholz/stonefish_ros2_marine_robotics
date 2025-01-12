// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from cola2_msgs:srv/Goto.idl
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
#include "cola2_msgs/srv/detail/goto__struct.h"
#include "cola2_msgs/srv/detail/goto__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool cola2_msgs__srv__goto__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[34];
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
    assert(strncmp("cola2_msgs.srv._goto.Goto_Request", full_classname_dest, 33) == 0);
  }
  cola2_msgs__srv__Goto_Request * ros_message = _ros_message;
  {  // final_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "final_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->final_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // final_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "final_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->final_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // final_depth
    PyObject * field = PyObject_GetAttrString(_pymsg, "final_depth");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->final_depth = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // final_altitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "final_altitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->final_altitude = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // reference
    PyObject * field = PyObject_GetAttrString(_pymsg, "reference");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->reference = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // heave_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "heave_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->heave_mode = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }
  {  // surge_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "surge_velocity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->surge_velocity = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // tolerance_xy
    PyObject * field = PyObject_GetAttrString(_pymsg, "tolerance_xy");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->tolerance_xy = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // timeout
    PyObject * field = PyObject_GetAttrString(_pymsg, "timeout");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->timeout = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // no_altitude_goes_up
    PyObject * field = PyObject_GetAttrString(_pymsg, "no_altitude_goes_up");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->no_altitude_goes_up = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * cola2_msgs__srv__goto__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Goto_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("cola2_msgs.srv._goto");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Goto_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  cola2_msgs__srv__Goto_Request * ros_message = (cola2_msgs__srv__Goto_Request *)raw_ros_message;
  {  // final_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->final_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "final_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // final_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->final_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "final_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // final_depth
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->final_depth);
    {
      int rc = PyObject_SetAttrString(_pymessage, "final_depth", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // final_altitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->final_altitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "final_altitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // reference
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->reference);
    {
      int rc = PyObject_SetAttrString(_pymessage, "reference", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // heave_mode
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->heave_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "heave_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // surge_velocity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->surge_velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "surge_velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tolerance_xy
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->tolerance_xy);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tolerance_xy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // timeout
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->timeout);
    {
      int rc = PyObject_SetAttrString(_pymessage, "timeout", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // no_altitude_goes_up
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->no_altitude_goes_up ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "no_altitude_goes_up", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "cola2_msgs/srv/detail/goto__struct.h"
// already included above
// #include "cola2_msgs/srv/detail/goto__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool cola2_msgs__srv__goto__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[35];
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
    assert(strncmp("cola2_msgs.srv._goto.Goto_Response", full_classname_dest, 34) == 0);
  }
  cola2_msgs__srv__Goto_Response * ros_message = _ros_message;
  {  // success
    PyObject * field = PyObject_GetAttrString(_pymsg, "success");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->success = (Py_True == field);
    Py_DECREF(field);
  }
  {  // message
    PyObject * field = PyObject_GetAttrString(_pymsg, "message");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->message, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * cola2_msgs__srv__goto__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Goto_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("cola2_msgs.srv._goto");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Goto_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  cola2_msgs__srv__Goto_Response * ros_message = (cola2_msgs__srv__Goto_Response *)raw_ros_message;
  {  // success
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->success ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "success", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // message
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->message.data,
      strlen(ros_message->message.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "message", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
