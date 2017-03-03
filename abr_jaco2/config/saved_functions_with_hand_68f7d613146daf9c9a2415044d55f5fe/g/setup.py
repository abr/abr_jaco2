from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
import numpy as np

setup(
    cmdclass = {'build_ext': build_ext},
    ext_modules = [Extension('wrapper_module_4', ['wrapper_module_4.pyx', 'wrapped_code_4.c'],
                             extra_compile_args=['-std=c99'])],
    include_dirs = [np.get_include()],
        )