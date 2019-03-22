# Contribution Guide

## Git Repository

 - Branches should be used actively to split development of different packages or different ROS version.
 - After validation or debug in a branch, a merge action to its mother branch should be taken in time. This branch should be removed if no need anymore. A merge action from mother branch is necessary before start new functionality development, if the branch will be kept further.
 - It is not recommanded to commit directly in master branch (protected), except editing text files.
 - It is encouraged to keep git history clean. Recommanded workfolw: pull remote master branch -> create new branch for feature/bug -> your work -> validation and test -> Squash and Rebase -> merge to master -> push to remote master branch. (https://blog.carbonfive.com/2017/08/28/always-squash-and-rebase-your-git-commits/)

## Code Style

The code style follows the [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide). Hence, class member variables have a underscore suffix (e.g. `variable1_`). Global variables have a leading `g_` prefix (e.g. `g_variable2`). For performance reasons, functions which are called frequently do not return values, but get a reference on the output passed as argument, so the result can be stored in this reference. This is a commonly used principle in C++.

It is recommanded to use clang-format to format your code automatically:
 - Install clang-format
 ```
 sudo apt-get install clang-format
 ```
 - Create a file `your_ws/.clang-format`, then paste the following style setting and save

 ```
 Language: Cpp
 BasedOnStyle: Google
 AccessModifierOffset: -2
 AlignEscapedNewlines: Left
 AlignTrailingComments: true
 AllowAllParametersOfDeclarationOnNextLine: false
 AllowShortIfStatementsOnASingleLine: false
 AllowShortLoopsOnASingleLine: false
 AllowShortFunctionsOnASingleLine: None
 AlwaysBreakTemplateDeclarations: true
 AlwaysBreakBeforeMultilineStrings: false
 BreakBeforeBinaryOperators: false
 BreakBeforeTernaryOperators: false
 BreakConstructorInitializers: BeforeComma
 BinPackParameters: true
 ColumnLimit: 120
 ConstructorInitializerAllOnOneLineOrOnePerLine: true
 ConstructorInitializerIndentWidth: 2
 ExperimentalAutoDetectBinPacking: false
 IndentCaseLabels: true
 MaxEmptyLinesToKeep: 1
 NamespaceIndentation: None
 ObjCSpaceBeforeProtocolList: true
 PenaltyBreakBeforeFirstCallParameter: 19
 PenaltyBreakComment: 60
 PenaltyBreakString: 1
 PenaltyBreakFirstLessLess: 1000
 PenaltyExcessCharacter: 1000
 PenaltyReturnTypeOnItsOwnLine: 90
 SpacesBeforeTrailingComments: 2
 Cpp11BracedListStyle: true
 Standard: Cpp11
 IndentWidth: 2
 TabWidth: 2
 UseTab: Never
 SpacesInParentheses: false
 SpacesInAngles:  false
 SpaceInEmptyParentheses: false
 SpacesInCStyleCastParentheses: false
 SpaceBeforeAssignmentOperators: true
 ContinuationIndentWidth: 4
 SortIncludes: false
 SpaceAfterCStyleCast: false
 BreakBeforeBraces: Custom
 BraceWrapping: 
     AfterClass: true
     AfterControlStatement: true 
     AfterEnum: true 
     AfterFunction: true 
     AfterNamespace: true 
     AfterStruct: true 
     AfterUnion: true 
     BeforeCatch: true 
     BeforeElse: true 
     IndentBraces: false 
 ```
 - Enable clang format using this style. Following the instruction [here](https://github.com/davetcoleman/roscpp_code_format/blob/master/README.md).

## Version Numbering Rules
Given a version number MAJOR.MINOR.PATCH, increment the:

 - MAJOR version when you make incompatible API changes,
 - MINOR version when you add functionality in a backwards-compatible manner, and
 - PATCH version when you make backwards-compatible bug fixes.
 
 Refer to [Semantic Versioning](https://semver.org/) for more details.
 