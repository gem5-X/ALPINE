# gem5-X + ALPINE

Analog In-Memory Compute (AIMC) tiles can be used to perform Matrix-Vector Multiplication (MVM) operations in constant time to overcome bottlenecks in machine learning, and in particular deep learning, applications.  Using gem5-X, we implement the ALPINE framework: a framework for simulating full systems using AIMC tiles that are tightly-coupled with their nearby CPU core.  One can easily implement AIMC tile-enabled applications using AIMClib: a header-only C++ library for interfacing the AIMC tiles using wrappers for intrinsics as well as higher-level functions for queueing and dequeueing larger data structures of various types.

The ALPINE framework, its features, description of the validated tile model, and case studies using ALPINE are described in the following paper:
>J. Klein, I. Boybat, Y. Qureshi, M. Dazzi, A. Levisse, G. Ansaloni, M. Zapater, A. Sebastian, and D. Atienza.
>"[**ALPINE: Analog In-Memory Acceleration with Tight Processor Integration for Deep Learning**]"(https://infoscience.epfl.ch/record/293272).
>_IEEE Transactions on Computers (TC)_, 2023.

## Installing and Compiling gem5-X

You can follow Section 2 of [this document](https://www.epfl.ch/labs/esl/wp-content/uploads/2021/08/gem5_X_TechnicalManual_v2.pdf) to install gem5-X.

## Compiling and creating benchmarks using AIMClib.

The contents of AIMClib are found in the *aimclib* folder.  An example perceptron application using AIMClib is in aimclib/example.cc, which can be compiled with the given makefile.  Compiling with the "-DUSE_CHECKER" option creates a binary that is meant to be run on a host system and uses AIMClib's emulation model to model the behaviour of the AIMC tiles in ALPINE.  When not using the checker option, the benchmark should be cross-compiled for the ARMv8 ISA.

## Running ALPINE simulations

No additional script options are required to run ALPINE simulations; your system will automatically spawn with one AIMC tile per CPU, parameterized according to the given size in gem5-X-ALPINE/src/dev/arm/aimc_cluster.hh.  An example command to run the gem5 binary is as follows:

```
./build/ARM/gem5.{fast, opt, debug} \
--remote-gdb-port=0 \
-d /path/to/your/output/directory \
configs/example/fs.py \
--cpu-clock=1GHz \
--kernel=vmlinux \
--machine-type=VExpress_GEM5_V1 \
--dtb-file=<path_to_gem5-X>/system/arm/dt/armv8_gem5_v1_<NUM_CORES>cpu.dtb \ 
-n <NUM_OF_CORES> \
--disk-image=gem5_ubuntu16.img  \
--caches  \
--l2cache  \
--l1i_size=32kB \
--l1d_size=32kB  \
--l2_size=1MB  \
--l2_assoc=2  \
--mem-type=DDR4_2400_4x16 \
--mem-ranks=4 \
--mem-size=4GB \
--sys-clock=1600MHz
```

## Acknowledgements

We thank Geethan Karunaratne, Pier Andrea Francese and Riduan Khaddam-Aljameh for technical discussions. This  work  has  been  supported  by  the  EC  H2020 WiPLASH  (GA  No.  863337) project and the ERC Consolidator Grant COMPUSAPIEN (GA No. 725657) projects.
