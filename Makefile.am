# Put compiled object files into the same sub-directory as their sources
AUTOMAKE_OPTIONS = subdir-objects

# Point to our macro directory and pick up user flags from the environment
ACLOCAL_AMFLAGS  = -I macros ${ACLOCAL_FLAGS}

# Make warnings always fatal at distcheck time, and disable execution
# of tools that change files not installed by the package itself
DISTCHECK_CONFIGURE_FLAGS = --enable-warnings=fatal --disable-update-mime-database

pkgconfigdir = $(libdir)/pkgconfig
nodist_pkgconfig_DATA = $(MURRAYC_SUFFIX_TREE_MODULE_NAME).pc

AM_CPPFLAGS = -I$(top_builddir) -I$(top_srcdir) 

COMMON_CXXFLAGS = $(PROJECT_CFLAGS) \
  $(PROJECT_WXXFLAGS) \
  $(BOOST_CPPFLAGS)
COMMON_LIBS = $(PROJECT_LIBS) \
  $(BOOST_SYSTEM_LIB) \
  $(BOOST_TIMER_LIB)

# TODO: Why do we still need to specify this for each executable that we build?
AM_CXXFLAGS = $(COMMON_CXXFLAGS)

check_PROGRAMS = \
  tests/test_trie \
  tests/test_radix_tree \
  tests/test_suffix_tree

TESTS = $(check_PROGRAMS)

#List of source files needed to build the executable:
tests_test_trie_SOURCES = \
	murrayc-suffix-tree/trie.h \
	tests/test_trie.cc
tests_test_trie_CXXFLAGS = \
	$(COMMON_CXXFLAGS)
tests_test_trie_LDADD = \
	$(COMMON_LIBS)

tests_test_radix_tree_SOURCES = \
	murrayc-suffix-tree/radix_tree.h \
	tests/test_radix_tree.cc
tests_test_radix_tree_CXXFLAGS = \
	$(COMMON_CXXFLAGS)
tests_test_radix_tree_LDADD = \
	$(COMMON_LIBS)

tests_test_suffix_tree_SOURCES = \
	murrayc-suffix-tree/suffix_tree.h \
	murrayc-suffix-tree/iter_range.h \
	tests/test_suffix_tree.cc
tests_test_suffix_tree_CXXFLAGS = \
	$(COMMON_CXXFLAGS)
tests_test_suffix_tree_LDADD = \
	$(COMMON_LIBS)
