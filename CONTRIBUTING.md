
# Contribution Guide

## Git branches

 - Branches should be used actively to split development of different packages or different ROS version.
 - After validation or debug in a branch, a merge action to its mother branch should be taken in time. This branch should be removed if no need anymore. A merge action from mother branch is necessary before start new functionality development, if the branch will be kept further.
 - It is not recommanded to commit directly in master branch (protected), except editing text files.

## Code Style

The code style follows the [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide). Hence, class member variables have a underscore suffix (e.g. `variable1_`). Global variables have a leading `g_` prefix (e.g. `g_variable2`). For performance reasons, functions which are called frequently do not return values, but get a reference on the output passed as argument, so the result can be stored in this reference. This is a commonly used principle in C++.

## Version Numbering Rules
Given a version number MAJOR.MINOR.PATCH, increment the:

 - MAJOR version when you make incompatible API changes,
 - MINOR version when you add functionality in a backwards-compatible manner, and
 - PATCH version when you make backwards-compatible bug fixes.
 
 Refer to [Semantic Versioning](https://semver.org/) for more details.
 