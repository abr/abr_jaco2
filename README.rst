ABR Jaco2 interface and config
==============================

A repository for the Jaco2 robot arm interface and configuration files, intended for use with the abr/control repo.

To run, first the Python shared library object must be created. 

Change directory to abr_jaco2/interface_files and run 

python setup.py build_ext -i

Second, the Kinova.API.CommLayerUbuntu.so file must be moved to whichever directory contains the script you are running. 
