from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
import numpy as np

setup(
    cmdclass = {'build_ext': build_ext},
    ext_modules = [Extension('wrapper_module_7', ['wrapper_module_7.pyx', 'wrapped_code_7.c'],
                             extra_compile_args=['-std=c99'])],
    include_dirs = [np.get_include()],
        )