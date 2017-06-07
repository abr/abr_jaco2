import io
import runpy
import os
import numpy as np

from setuptools import Extension, find_packages, setup
from setuptools.command.build_ext import build_ext as _build_ext


class build_ext(_build_ext):
    def finalize_options(self):
        _build_ext.finalize_options(self)
        # Prevent numpy from thinking it is still in its setup process:
        __builtins__.__NUMPY_SETUP__ = False
        import numpy
        self.include_dirs.append(numpy.get_include())

class build_ext(_build_ext):
    def finalize_options(self):
        _build_ext.finalize_options(self)
        # Prevent numpy from thinking it is still in its setup process:
        __builtins__.__NUMPY_SETUP__ = False
        import numpy
        self.include_dirs.append(numpy.get_include())

def read(*filenames, **kwargs):
    encoding = kwargs.get('encoding', 'utf-8')
    sep = kwargs.get('sep', '\n')
    buf = []
    for filename in filenames:
        with io.open(filename, encoding=encoding) as f:
            buf.append(f.read())
    return sep.join(buf)


root = os.path.dirname(os.path.realpath(__file__))
version = runpy.run_path(
    os.path.join(root, 'abr_jaco2', 'version.py'))['version']
setup_requires = ["setuptools>=18.0", "cython", "numpy"]

setup(
    name="abr_jaco2",
    version=version,
    author="Applied Brain Research",
    author_email="travis.dewolf@appliedbrainresearch.com",
    packages=find_packages(),
    include_package_data=True,
    scripts=[],
    url="https://github.com/abr/jaco2",
    license="Free for non-commercial use",
    description="ABR Jaco2 interface and config",
    long_description=read('README.rst'),
    install_requires=setup_requires + [
        "sympy", "cloudpickle",
    ],
    setup_requires=setup_requires,
    cmdclass = {'build_ext': build_ext},
    ext_modules=[
        Extension(
            "abr_jaco2.interface.jaco2_rs485",
            sources=["abr_jaco2/interface/jaco2_cython.pyx",
                     "abr_jaco2/interface/jaco2_rs485.cpp"],
            language="c++",
            include_dirs=[np.get_include()],
            extra_compile_args=["-ldl"],
        )
    ],
)
