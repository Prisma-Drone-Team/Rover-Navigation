// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from roboclaw_ros2:msg/RoboclawMotorVelocity.idl
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
#include "roboclaw_ros2/msg/detail/roboclaw_motor_velocity__struct.h"
#include "roboclaw_ros2/msg/detail/roboclaw_motor_velocity__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool roboclaw_ros2__msg__roboclaw_motor_velocity__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[65];
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
    assert(strncmp("roboclaw_ros2.msg._roboclaw_motor_velocity.RoboclawMotorVelocity", full_classname_dest, 64) == 0);
  }
  roboclaw_ros2__msg__RoboclawMotorVelocity * ros_message = _ros_message;
  {  // index
    PyObject * field = PyObject_GetAttrString(_pymsg, "index");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->index = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // mot1_vel_sps
    PyObject * field = PyObject_GetAttrString(_pymsg, "mot1_vel_sps");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->mot1_vel_sps = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // mot2_vel_sps
    PyObject * field = PyObject_GetAttrString(_pymsg, "mot2_vel_sps");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->mot2_vel_sps = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * roboclaw_ros2__msg__roboclaw_motor_velocity__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of RoboclawMotorVelocity */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("roboclaw_ros2.msg._roboclaw_motor_velocity");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "RoboclawMotorVelocity");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  roboclaw_ros2__msg__RoboclawMotorVelocity * ros_message = (roboclaw_ros2__msg__RoboclawMotorVelocity *)raw_ros_message;
  {  // index
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->index);
    {
      int rc = PyObject_SetAttrString(_pymessage, "index", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // mot1_vel_sps
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->mot1_vel_sps);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mot1_vel_sps", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // mot2_vel_sps
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->mot2_vel_sps);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mot2_vel_sps", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
