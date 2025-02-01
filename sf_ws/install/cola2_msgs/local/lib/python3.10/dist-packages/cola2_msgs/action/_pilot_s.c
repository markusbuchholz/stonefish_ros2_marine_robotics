// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from cola2_msgs:action/Pilot.idl
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
#include "cola2_msgs/action/detail/pilot__struct.h"
#include "cola2_msgs/action/detail/pilot__functions.h"

bool cola2_msgs__msg__goal_descriptor__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * cola2_msgs__msg__goal_descriptor__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool cola2_msgs__action__pilot__goal__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[36];
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
    assert(strncmp("cola2_msgs.action._pilot.Pilot_Goal", full_classname_dest, 35) == 0);
  }
  cola2_msgs__action__Pilot_Goal * ros_message = _ros_message;
  {  // initial_latitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "initial_latitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->initial_latitude = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // initial_longitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "initial_longitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->initial_longitude = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // initial_depth
    PyObject * field = PyObject_GetAttrString(_pymsg, "initial_depth");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->initial_depth = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // final_latitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "final_latitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->final_latitude = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // final_longitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "final_longitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->final_longitude = PyFloat_AS_DOUBLE(field);
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
  {  // final_yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "final_yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->final_yaw = PyFloat_AS_DOUBLE(field);
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
  {  // controller_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "controller_type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->controller_type = PyLong_AsUnsignedLongLong(field);
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

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * cola2_msgs__action__pilot__goal__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Pilot_Goal */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("cola2_msgs.action._pilot");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Pilot_Goal");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  cola2_msgs__action__Pilot_Goal * ros_message = (cola2_msgs__action__Pilot_Goal *)raw_ros_message;
  {  // initial_latitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->initial_latitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "initial_latitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // initial_longitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->initial_longitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "initial_longitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // initial_depth
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->initial_depth);
    {
      int rc = PyObject_SetAttrString(_pymessage, "initial_depth", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // final_latitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->final_latitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "final_latitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // final_longitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->final_longitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "final_longitude", field);
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
  {  // final_yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->final_yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "final_yaw", field);
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
  {  // controller_type
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->controller_type);
    {
      int rc = PyObject_SetAttrString(_pymessage, "controller_type", field);
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
// #include "cola2_msgs/action/detail/pilot__struct.h"
// already included above
// #include "cola2_msgs/action/detail/pilot__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool cola2_msgs__action__pilot__result__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[38];
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
    assert(strncmp("cola2_msgs.action._pilot.Pilot_Result", full_classname_dest, 37) == 0);
  }
  cola2_msgs__action__Pilot_Result * ros_message = _ros_message;
  {  // state
    PyObject * field = PyObject_GetAttrString(_pymsg, "state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->state = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * cola2_msgs__action__pilot__result__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Pilot_Result */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("cola2_msgs.action._pilot");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Pilot_Result");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  cola2_msgs__action__Pilot_Result * ros_message = (cola2_msgs__action__Pilot_Result *)raw_ros_message;
  {  // state
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "state", field);
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
// #include "cola2_msgs/action/detail/pilot__struct.h"
// already included above
// #include "cola2_msgs/action/detail/pilot__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool cola2_msgs__action__pilot__feedback__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[40];
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
    assert(strncmp("cola2_msgs.action._pilot.Pilot_Feedback", full_classname_dest, 39) == 0);
  }
  cola2_msgs__action__Pilot_Feedback * ros_message = _ros_message;
  {  // distance_to_end
    PyObject * field = PyObject_GetAttrString(_pymsg, "distance_to_end");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->distance_to_end = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cross_track_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "cross_track_error");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cross_track_error = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * cola2_msgs__action__pilot__feedback__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Pilot_Feedback */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("cola2_msgs.action._pilot");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Pilot_Feedback");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  cola2_msgs__action__Pilot_Feedback * ros_message = (cola2_msgs__action__Pilot_Feedback *)raw_ros_message;
  {  // distance_to_end
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->distance_to_end);
    {
      int rc = PyObject_SetAttrString(_pymessage, "distance_to_end", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cross_track_error
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cross_track_error);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cross_track_error", field);
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
// #include "cola2_msgs/action/detail/pilot__struct.h"
// already included above
// #include "cola2_msgs/action/detail/pilot__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool unique_identifier_msgs__msg__uuid__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * unique_identifier_msgs__msg__uuid__convert_to_py(void * raw_ros_message);
bool cola2_msgs__action__pilot__goal__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * cola2_msgs__action__pilot__goal__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool cola2_msgs__action__pilot__send_goal__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[48];
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
    assert(strncmp("cola2_msgs.action._pilot.Pilot_SendGoal_Request", full_classname_dest, 47) == 0);
  }
  cola2_msgs__action__Pilot_SendGoal_Request * ros_message = _ros_message;
  {  // goal_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "goal_id");
    if (!field) {
      return false;
    }
    if (!unique_identifier_msgs__msg__uuid__convert_from_py(field, &ros_message->goal_id)) {
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
    if (!cola2_msgs__action__pilot__goal__convert_from_py(field, &ros_message->goal)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * cola2_msgs__action__pilot__send_goal__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Pilot_SendGoal_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("cola2_msgs.action._pilot");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Pilot_SendGoal_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  cola2_msgs__action__Pilot_SendGoal_Request * ros_message = (cola2_msgs__action__Pilot_SendGoal_Request *)raw_ros_message;
  {  // goal_id
    PyObject * field = NULL;
    field = unique_identifier_msgs__msg__uuid__convert_to_py(&ros_message->goal_id);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "goal_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // goal
    PyObject * field = NULL;
    field = cola2_msgs__action__pilot__goal__convert_to_py(&ros_message->goal);
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
// #include "cola2_msgs/action/detail/pilot__struct.h"
// already included above
// #include "cola2_msgs/action/detail/pilot__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool builtin_interfaces__msg__time__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * builtin_interfaces__msg__time__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool cola2_msgs__action__pilot__send_goal__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[49];
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
    assert(strncmp("cola2_msgs.action._pilot.Pilot_SendGoal_Response", full_classname_dest, 48) == 0);
  }
  cola2_msgs__action__Pilot_SendGoal_Response * ros_message = _ros_message;
  {  // accepted
    PyObject * field = PyObject_GetAttrString(_pymsg, "accepted");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->accepted = (Py_True == field);
    Py_DECREF(field);
  }
  {  // stamp
    PyObject * field = PyObject_GetAttrString(_pymsg, "stamp");
    if (!field) {
      return false;
    }
    if (!builtin_interfaces__msg__time__convert_from_py(field, &ros_message->stamp)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * cola2_msgs__action__pilot__send_goal__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Pilot_SendGoal_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("cola2_msgs.action._pilot");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Pilot_SendGoal_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  cola2_msgs__action__Pilot_SendGoal_Response * ros_message = (cola2_msgs__action__Pilot_SendGoal_Response *)raw_ros_message;
  {  // accepted
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->accepted ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "accepted", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // stamp
    PyObject * field = NULL;
    field = builtin_interfaces__msg__time__convert_to_py(&ros_message->stamp);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "stamp", field);
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
// #include "cola2_msgs/action/detail/pilot__struct.h"
// already included above
// #include "cola2_msgs/action/detail/pilot__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool unique_identifier_msgs__msg__uuid__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * unique_identifier_msgs__msg__uuid__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool cola2_msgs__action__pilot__get_result__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[49];
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
    assert(strncmp("cola2_msgs.action._pilot.Pilot_GetResult_Request", full_classname_dest, 48) == 0);
  }
  cola2_msgs__action__Pilot_GetResult_Request * ros_message = _ros_message;
  {  // goal_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "goal_id");
    if (!field) {
      return false;
    }
    if (!unique_identifier_msgs__msg__uuid__convert_from_py(field, &ros_message->goal_id)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * cola2_msgs__action__pilot__get_result__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Pilot_GetResult_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("cola2_msgs.action._pilot");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Pilot_GetResult_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  cola2_msgs__action__Pilot_GetResult_Request * ros_message = (cola2_msgs__action__Pilot_GetResult_Request *)raw_ros_message;
  {  // goal_id
    PyObject * field = NULL;
    field = unique_identifier_msgs__msg__uuid__convert_to_py(&ros_message->goal_id);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "goal_id", field);
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
// #include "cola2_msgs/action/detail/pilot__struct.h"
// already included above
// #include "cola2_msgs/action/detail/pilot__functions.h"

bool cola2_msgs__action__pilot__result__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * cola2_msgs__action__pilot__result__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool cola2_msgs__action__pilot__get_result__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[50];
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
    assert(strncmp("cola2_msgs.action._pilot.Pilot_GetResult_Response", full_classname_dest, 49) == 0);
  }
  cola2_msgs__action__Pilot_GetResult_Response * ros_message = _ros_message;
  {  // status
    PyObject * field = PyObject_GetAttrString(_pymsg, "status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->status = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // result
    PyObject * field = PyObject_GetAttrString(_pymsg, "result");
    if (!field) {
      return false;
    }
    if (!cola2_msgs__action__pilot__result__convert_from_py(field, &ros_message->result)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * cola2_msgs__action__pilot__get_result__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Pilot_GetResult_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("cola2_msgs.action._pilot");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Pilot_GetResult_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  cola2_msgs__action__Pilot_GetResult_Response * ros_message = (cola2_msgs__action__Pilot_GetResult_Response *)raw_ros_message;
  {  // status
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // result
    PyObject * field = NULL;
    field = cola2_msgs__action__pilot__result__convert_to_py(&ros_message->result);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "result", field);
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
// #include "cola2_msgs/action/detail/pilot__struct.h"
// already included above
// #include "cola2_msgs/action/detail/pilot__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool unique_identifier_msgs__msg__uuid__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * unique_identifier_msgs__msg__uuid__convert_to_py(void * raw_ros_message);
bool cola2_msgs__action__pilot__feedback__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * cola2_msgs__action__pilot__feedback__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool cola2_msgs__action__pilot__feedback_message__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("cola2_msgs.action._pilot.Pilot_FeedbackMessage", full_classname_dest, 46) == 0);
  }
  cola2_msgs__action__Pilot_FeedbackMessage * ros_message = _ros_message;
  {  // goal_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "goal_id");
    if (!field) {
      return false;
    }
    if (!unique_identifier_msgs__msg__uuid__convert_from_py(field, &ros_message->goal_id)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // feedback
    PyObject * field = PyObject_GetAttrString(_pymsg, "feedback");
    if (!field) {
      return false;
    }
    if (!cola2_msgs__action__pilot__feedback__convert_from_py(field, &ros_message->feedback)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * cola2_msgs__action__pilot__feedback_message__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Pilot_FeedbackMessage */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("cola2_msgs.action._pilot");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Pilot_FeedbackMessage");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  cola2_msgs__action__Pilot_FeedbackMessage * ros_message = (cola2_msgs__action__Pilot_FeedbackMessage *)raw_ros_message;
  {  // goal_id
    PyObject * field = NULL;
    field = unique_identifier_msgs__msg__uuid__convert_to_py(&ros_message->goal_id);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "goal_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // feedback
    PyObject * field = NULL;
    field = cola2_msgs__action__pilot__feedback__convert_to_py(&ros_message->feedback);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "feedback", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
