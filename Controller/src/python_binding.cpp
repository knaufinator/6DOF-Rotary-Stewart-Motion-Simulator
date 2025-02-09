#include <Python.h>
#include <numpy/arrayobject.h>
#include "../include/helpers.h"

// Create a Python dictionary with all platform parameters
static PyObject* get_platform_params(void) {
    PyObject* dict = PyDict_New();
    
    // Add all parameters from helpers.h
    PyDict_SetItemString(dict, "theta_r", PyFloat_FromDouble(theta_r));
    
    // Create theta_s array
    PyObject* theta_s_list = PyList_New(6);
    for(int i = 0; i < 6; i++) {
        PyList_SetItem(theta_s_list, i, PyFloat_FromDouble(theta_s[i]));
    }
    PyDict_SetItemString(dict, "theta_s", theta_s_list);
    
    PyDict_SetItemString(dict, "theta_p", PyFloat_FromDouble(theta_p));
    PyDict_SetItemString(dict, "RD", PyFloat_FromDouble(RD));
    PyDict_SetItemString(dict, "PD", PyFloat_FromDouble(PD));
    PyDict_SetItemString(dict, "servo_arm_length", PyFloat_FromDouble(ServoArmLengthL1));
    PyDict_SetItemString(dict, "connecting_arm_length", PyFloat_FromDouble(ConnectingArmLengthL2));
    PyDict_SetItemString(dict, "platform_height", PyFloat_FromDouble(platformHeight));
    PyDict_SetItemString(dict, "servo_min", PyFloat_FromDouble(servo_min));
    PyDict_SetItemString(dict, "servo_max", PyFloat_FromDouble(servo_max));
    
    return dict;
}

// Wrapper for getAlpha function
static PyObject* py_get_alpha(PyObject* self, PyObject* args) {
    int motor_index;
    PyObject* position_array;
    
    // Parse Python arguments
    if (!PyArg_ParseTuple(args, "iO", &motor_index, &position_array)) {
        return NULL;
    }
    
    // Convert Python list to C array
    float position[6];
    for (int i = 0; i < 6; i++) {
        PyObject* item = PyList_GetItem(position_array, i);
        position[i] = (float)PyFloat_AsDouble(item);
    }
    
    // Call the C function
    float result = getAlpha(motor_index, position);
    
    // Return the result
    return PyFloat_FromDouble(result);
}

// Method definitions
static PyMethodDef StewartMethods[] = {
    {"get_alpha", py_get_alpha, METH_VARARGS, "Calculate servo angle for given motor and position"},
    {"get_platform_params", (PyCFunction)get_platform_params, METH_NOARGS, "Get all platform parameters"},
    {NULL, NULL, 0, NULL}
};

// Module definition
static struct PyModuleDef stewartmodule = {
    PyModuleDef_HEAD_INIT,
    "stewart_core",
    "Python interface for Stewart platform core calculations",
    -1,
    StewartMethods
};

// Module initialization function
PyMODINIT_FUNC PyInit_stewart_core(void) {
    import_array();  // Initialize NumPy
    return PyModule_Create(&stewartmodule);
}
