AC_INIT([murrayc-suffix-tree],[0.0.1])
AC_PREREQ(2.63)

AC_CONFIG_SRCDIR([murrayc-suffix-tree/suffix_tree.h])
AC_CONFIG_MACRO_DIR([macros])
$AC_CONFIG_HEADERS([config.h])

AM_INIT_AUTOMAKE([1.10 -Wno-portability no-define nostdinc tar-ustar])
m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])
AM_MAINTAINER_MODE
AC_ARG_VAR([ACLOCAL_FLAGS], [aclocal flags, e.g. -I <macro dir>])

AC_PROG_CC
AM_PROG_CC_C_O
AC_PROG_CXX

AX_CXX_COMPILE_STDCXX([14], [noext],[mandatory])


//PKG_CHECK_MODULES([PROJECT], [gdkmm-3.0])

# See http://www.gnu.org/software/autoconf-archive/ax_boost_base.html
# Boost System is needed by Boost Timer
AX_BOOST_BASE
AX_BOOST_SYSTEM
AX_BOOST_TIMER

# Allow use of mm-common macros for the warnings option.
MM_PREREQ([0.9.5])
MM_INIT_MODULE([murrayc-suffix-tree-0-1])

# http://www.gnu.org/software/libtool/manual/html_node/Updating-version-info.html
AC_SUBST([LIBMURRAYC_SUFFIX_TREE_SO_VERSION], [0:0:0])

AC_LANG([C++])
MM_ARG_ENABLE_WARNINGS([PROJECT_WXXFLAGS],
                       [-Wall],
                       [-pedantic -Wall -Wextra -Wsuggest-override -Wshadow -Wformat-security -Wno-missing-field-initializers],
                       [])


AC_CONFIG_FILES([Makefile
                 ${MURRAYC_SUFFIX_TREE_MODULE_NAME}.pc:murrayc-suffix-tree.pc.in])

AC_OUTPUT
