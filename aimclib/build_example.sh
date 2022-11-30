#!/bin/sh

# Build example.cc with checker.
g++ -O3 -DUSE_CHECKER example.cc -o example_checker.out

# Build example.cc without checker.
g++ -O3 example.cc -o example_gem5.out

echo "Done!"
