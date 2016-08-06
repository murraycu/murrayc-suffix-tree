# Put compiled object files into the same sub-directory as their sources
AUTOMAKE_OPTIONS = subdir-objects

# Point to our macro directory and pick up user flags from the environment
ACLOCAL_AMFLAGS  = -I macros ${ACLOCAL_FLAGS}

# Make warnings always fatal at distcheck time, and disable execution
# of tools that change files not installed by the package itself
DISTCHECK_CONFIGURE_FLAGS = --enable-warnings=fatal --disable-update-mime-database

#SUBDIRS =


COMMON_CXXFLAGS = $(PROJECT_CFLAGS) \
  $(PROJECT_WXXFLAGS) \
  $(BOOST_CPPFLAGS)
COMMON_LIBS = $(PROJECT_LIBS) \
  $(BOOST_SYSTEM_LIB) \
  $(BOOST_TIMER_LIB)

# TODO: Why do we still need to specify this for each executable that we build?
AM_CXXFLAGS = $(COMMON_CXXFLAGS)

bin_PROGRAMS = \
  trie \
  radix_tree \
  suffix_tree

#List of source files needed to build the executable:
trie_SOURCES = \
	src/trees/trie/trie.cc
trie_CXXFLAGS = \
	$(COMMON_CXXFLAGS)
trie_LDADD = \
	$(COMMON_LIBS)

radix_tree_SOURCES = \
	src/trees/radix_tree/radix_tree.h \
	src/trees/radix_tree/main.cc
radix_tree_CXXFLAGS = \
	$(COMMON_CXXFLAGS)
radix_tree_LDADD = \
	$(COMMON_LIBS)

suffix_tree_SOURCES = \
	src/trees/radix_tree/radix_tree.h \
	src/trees/suffix_tree/suffix_tree.h \
	src/trees/suffix_tree/main.cc
suffix_tree_CXXFLAGS = \
	$(COMMON_CXXFLAGS)
suffix_tree_LDADD = \
	$(COMMON_LIBS)
