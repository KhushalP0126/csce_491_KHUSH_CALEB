# Copyright 2021 Jason Bakos, Philip Conrad, Charles Daniels
#
# Distributed as part of the University of South Carolina CSCE491 course
# materials. Please do not redistribute without written authorization.

C_UTIL_DIR=$(shell pwd)/../utils/c_utils
CC=gcc
CXX=g++
CFLAGS=-Wall -Wextra -pedantic -Wmisleading-indentation -Werror -std=gnu11 -g -Og
CXXFLAGS=-Wall -Wextra -pedantic -Wmisleading-indentation -Werror -std=gnu++17 -g -Og
CINCLUDE=-lm -I$(C_UTIL_DIR)

sources_cpp = $(wildcard *.cpp)
headers = $(wildcard *.h)
objects = $(patsubst %.cpp,%.o,$(sources_cpp))

a.out: $(objects) waves.o
	$(CXX) $(CXXFLAGS) $^ $(CINCLUDE) -o $@

%.o: %.cpp $(headers)
	$(CXX) $(CXXFLAGS) -c $< -o $@ $(CINCLUDE)

waves.o:
	$(MAKE) CFLAGS="$(CFLAGS)" CINCLUDE="$(CINCLUDE)" CC="$(CC)" -C $(C_UTIL_DIR)
	cp $(C_UTIL_DIR)/waves.o ./

clean:
	rm -f $(objects) waves.o a.out
	$(MAKE) -C $(C_UTIL_DIR) clean
