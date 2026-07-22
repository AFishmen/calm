# Makefile
CXX = g++
CC = gcc
CXXFLAGS = -std=c++11 -Wall -O2
CFLAGS = -Wall -O2
LDFLAGS = -pthread -lm

all: calm_server

calm_server: main.o calm_hw.o calm_ctrl.o calm_net.o
	$(CXX) -o $@ $^ $(LDFLAGS)

main.o: main.cpp calm_ctrl.hpp calm_net.hpp
	$(CXX) $(CXXFLAGS) -c $<

calm_ctrl.o: calm_ctrl.cpp calm_ctrl.hpp calm_hw.h
	$(CXX) $(CXXFLAGS) -c $<

calm_net.o: calm_net.cpp calm_net.hpp calm_ctrl.hpp
	$(CXX) $(CXXFLAGS) -c $<

calm_hw.o: calm_hw.c calm_hw.h
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f *.o calm_server
