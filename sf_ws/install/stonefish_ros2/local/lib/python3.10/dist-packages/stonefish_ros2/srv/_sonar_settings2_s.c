// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from stonefish_ros2:srv/SonarSettings2.idl
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
#include "stonefish_ros2/srv/detail/sonar_settings2__struct.h"
#include "stonefish_ros2/srv/detail/sonar_settings2__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool stonefish_ros2__srv__sonar_settings2__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[59];
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
    assert(strncmp("stonefish_ros2.srv._sonar_settings2.SonarSettings2_Request", full_classname_dest, 58) == 0);
  }
  stonefish_ros2__srv__SonarSettings2_Request * ros_message = _ros_message;
  {  // range_min
    PyObject * field = PyObject_GetAttrString(_pymsg, "range_min");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->range_min = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // range_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "range_max");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->range_max = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rotation_min
    PyObject * field = PyObject_GetAttrString(_pymsg, "rotation_min");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rotation_min = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rotation_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "rotation_max");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rotation_max = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // gain
    PyObject * field = PyObject_GetAttrString(_pymsg, "gain");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->gain = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * stonefish_ros2__srv__sonar_settings2__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SonarSettings2_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("stonefish_ros2.srv._sonar_settings2");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SonarSettings2_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  stonefish_ros2__srv__SonarSettings2_Request * ros_message = (stonefish_ros2__srv__SonarSettings2_Request *)raw_ros_message;
  {  // range_min
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->range_min);
    {
      int rc = PyObject_SetAttrString(_pymessage, "range_min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // range_max
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->range_max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "range_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rotation_min
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rotation_min);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rotation_min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rotation_max
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rotation_max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rotation_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // gain
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->gain);
    {
      int rc = PyObject_SetAttrString(_pymessage, "gain", field);
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
// #include "stonefish_ros2/srv/detail/sonar_settings2__struct.h"
// already included above
// #include "stonefish_ros2/srv/detail/sonar_settings2__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool stonefish_ros2__srv__sonar_settings2__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[60];
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
    assert(strncmp("stonefish_ros2.srv._sonar_settings2.SonarSettings2_Response", full_classname_dest, 59) == 0);
  }
  stonefish_ros2__srv__SonarSettings2_Response * ros_message = _ros_message;
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
PyObject * stonefish_ros2__srv__sonar_settings2__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SonarSettings2_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("stonefish_ros2.srv._sonar_settings2");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SonarSettings2_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  stonefish_ros2__srv__SonarSettings2_Response * ros_message = (stonefish_ros2__srv__SonarSettings2_Response *)raw_ros_message;
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
