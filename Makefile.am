AUTOMAKE_OPTIONS=foreign

lib_LIBRARIES=lib/libmpu6050dmp.a

lib_libmpu6050dmp_a_SOURCES= \
    src/dmpKey.h \
    src/dmpmap.h \
    src/inv_mpu.c \
    src/inv_mpu_dmp_motion_driver.c \
    src/inv_mpu_dmp_motion_driver.h \
    src/mpumanager.cpp \
    src/mpumanager.h \
    src/inv_mpu.h

INCLUDES=-I/usr/include

CXXFLAGS = -O3 -Wall
CFLAGS = -O3 -Wall

include_HEADERS = src/inv_mpu_dmp_motion_driver.h src/inv_mpu.h src/mpumanager.h

EXTRA_DIST = \
	build.sh \
	README \
	example/aclocal.m4 \
	example/config.h.in \
	example/configure \
	example/configure.in \
	example/Makefile.am \
	example/Makefile.in \
	example/test.cpp

