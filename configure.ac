AC_PREREQ(2.50)

AC_INIT(bioinstsim, 0.7.0)
AM_INIT_AUTOMAKE(bioinstsim
, 0.7.0)
AM_CONFIG_HEADER(config.h)

dnl AC_CHECK_HEADERS(readline/readline.h, [], AC_MSG_ERROR([You need to install readline.h (probably in a readline-dev package)]))

dnl check for programs
AC_PROG_CC
AC_PROG_CXX
AC_PROG_INSTALL
AC_PROG_RANLIB

#CPPFLAGS="$CPPFLAGS -g3 -ggdb3 -Wall -Wno-write-strings -p"
#CXXFLAGS="$CXXFLAGS -g3 -ggdb3 -Wall -Wno-write-strings -p"
#CFLAGS="$CFLAGS -g3 -ggdb3 -Wall -Wno-write-strings -p"
CPPFLAGS="$CPPFLAGS -Wno-write-strings"
CXXFLAGS="$CXXFLAGS -Wno-write-strings"
CFLAGS="$CFLAGS -Wno-write-strings"

AC_CONFIG_FILES([ \
  Makefile        
])

AC_OUTPUT
                                                                               
