#!/bin/bash

#nano overlay_edges_fast.cpp
g++ -static -O2 -march=armv7-a -mfpu=neon -mfloat-abi=hard -fno-exceptions -fno-rtti   -o overlay_edges_fast overlay_edges_fast.cpp && strip overlay_edges_fast
