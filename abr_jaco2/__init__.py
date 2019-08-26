import sys

from .config.config import Config
from .interface.interface import Interface
from .interface import jaco2_rs485

if sys.version_info > (3, 6, 9):
    raise ImportError(
        """ You are using Python version %s and abr_jaco2
            currently supports python up to 3.6.9.
            Please create a new environment with python =<3.6.9
        """ % (sys.version))
