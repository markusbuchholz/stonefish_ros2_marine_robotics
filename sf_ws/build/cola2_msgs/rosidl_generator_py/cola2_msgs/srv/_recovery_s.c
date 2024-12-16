// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from cola2_msgs:srv/Recovery.idl
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
#include "cola2_msgs/srv/detail/recovery__struct.h"
#include "cola2_msgs/srv/detail/recovery__functions.h"

bool cola2_msgs__msg__recovery_action__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * cola2_msgs__msg__recovery_action__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool cola2_msgs__srv__recovery__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[42];
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
    assert(strncmp("cola2_msgs.srv._recovery.Recovery_Request", full_classname_dest, 41) == 0);
  }
  cola2_msgs__srv__Recovery_Request * ros_message = _ros_message;
  {  // requested_action
    PyObject * field = PyObject_GetAttrString(_pymsg, "requested_action");
    if (!field) {
      return false;
    }
    if (!cola2_msgs__msg__recovery_action__convert_from_py(field, &ros_message->requested_action)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * cola2_msgs__srv__recovery__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Recovery_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("cola2_msgs.srv._recovery");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Recovery_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  cola2_msgs__srv__Recovery_Request * ros_message = (cola2_msgs__srv__Recovery_Request *)raw_ros_message;
  {  // requested_action
    PyObject * field = NULL;
    field = cola2_msgs__msg__recovery_action__convert_to_py(&ros_message->requested_action);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "requested_action", field);
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
// #include "cola2_msgs/srv/detail/recovery__struct.h"
// already included above
// #include "cola2_msgs/srv/detail/recovery__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool cola2_msgs__srv__recovery__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("cola2_msgs.srv._recovery.Recovery_Response", full_classname_dest, 42) == 0);
  }
  cola2_msgs__srv__Recovery_Response * ros_message = _ros_message;
  {  // attempted
    PyObject * field = PyObject_GetAttrString(_pymsg, "attempted");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->attempted = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * cola2_msgs__srv__recovery__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Recovery_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("cola2_msgs.srv._recovery");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Recovery_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  cola2_msgs__srv__Recovery_Response * ros_message = (cola2_msgs__srv__Recovery_Response *)raw_ros_message;
  {  // attempted
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->attempted ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "attempted", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
