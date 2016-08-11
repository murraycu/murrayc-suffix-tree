# murrayc-suffix-tree

This experiment tries to provide a generic Suffix Tree implementation for C++.


## Basic Usage

Include the specific header. For instance,
```C++
#include <murrayc-suffix-tree/suffix_tree.h>
```

If your source file is program.cc, you can compile it with:
```shell
g++ program.cc -o program `pkg-config --cflags --libs murrayc-suffix-tree-1.0`
```

## Using Autotools

Alternatively, if using autoconf, use the following in configure.ac:
```m4
PKG_CHECK_MODULES([DEPS], [murrayc-suffix-tree-1.0])
```

Then use the generated DEPS_CFLAGS and DEPS_LIBS variables in the project Makefile.am files. For example:
```Makefile
yourprogram_CPPFLAGS = $(DEPS_CFLAGS)
yourprogram_LDADD = $(DEPS_LIBS)
```

Your PKG_CHECK_MODULES() call should also mention any other libraries that you need to use via pkg-config.

## Using CMake

If using CMake, use the following in CMakeList.txt:
```CMake
include(FindPkgConfig)
pkg_check_modules(DEPS REQUIRED murrayc-suffix-tree-1.0)
include_directories(${DEPS_INCLUDE_DIRS})
target_link_libraries(yourprogram ${DEPS_LIBRARIES})
```

Your pkg_check_modules() call should also mention any other libraries that you need to use via pkg-config
