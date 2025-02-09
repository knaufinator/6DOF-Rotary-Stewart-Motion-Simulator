from setuptools import setup, Extension
import numpy as np

stewart_module = Extension('stewart_core',
                         sources=['src/python_binding.cpp', 'src/helpers.cpp'],
                         include_dirs=[np.get_include(), 'include'],
                         extra_compile_args=['-std=c++11'])

setup(name='stewart_core',
      version='1.0',
      description='Python interface for Stewart platform core calculations',
      ext_modules=[stewart_module])
