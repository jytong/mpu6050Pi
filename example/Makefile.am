AUTOMAKE_OPTIONS=foreign

noinst_PROGRAMS=dmptest

dmptest_SOURCES=test.cpp

INCLUDES=-I/usr/local/include

CXXFLAGS = -O3 -Wall

dmptest_LDADD=-lpthread -lwiringPi -lm -lrt -L/usr/local/lib -lmpu6050dmp

