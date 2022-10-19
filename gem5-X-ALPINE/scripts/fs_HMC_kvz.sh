#export M5_PATH=$HOME/gem5


MAIN_CPUS=1
ALT_CPUS=4
NUM_CPUS=$(( $MAIN_CPUS + $ALT_CPUS ))
#DTB="/home/herruzo/gem5_ac.uma/system/arm/dt/armv8_gem5_v1_"$NUM_CPUS"cpu.dtb"
#DTB="$HOME/gem5_16_08_2018_HMC/system/arm/dt/armv8_gem5_v1_"$MAIN_CPUS"_"$ALT_CPUS".dtb"
#DTB="/home/yqureshi/shares/local/scrap/gem5_16_08_2018/system/arm/dt/armv8_gem5_v1_"$MAIN_CPUS"cpu.dtb"
DTB="/home/yqureshi/shares/local/scrap/gem5_16_08_2018/system/arm/dt/armv8_gem5_v1_"$MAIN_CPUS"_"$ALT_CPUS".dtb"
KERNEL=vmlinux
CPU_TYPE=AtomicSimpleCPU
ALT_CPU_TYPE=AtomicSimpleCPU
DISK=test_kvazaar.img
DDR_SIZE=4GB
#HMC_SIZE=2GB


build/ARM/gem5.fast --remote-gdb-port=0 configs/fs_HMC_DDR/fs.py --machine-type=VExpress_GEM5_V1  \
--kernel $KERNEL --dtb=$DTB --main-num-cpus=$MAIN_CPUS --cpu-clock=2GHz  \
--caches --cpu-type=$CPU_TYPE --disk=$DISK --alt-cpu-type=$ALT_CPU_TYPE \
--l2cache --l1i_size=32kB --l1d_size=32kB --l2_size=1MB --l2_assoc=2  \
--mem-type=DDR4_2400_4x16 --mem-ranks=4 --ddr-size=$DDR_SIZE  \
--alt-num-cpus=$ALT_CPUS  --sys-clock=1600MHz  \
--num-serial-links=5  \
--no_ddr  \
#--workload-automation-vio=/home/yqureshi/temp \
#--checkpoint-restore=1 --restore-with-cpu=$CPU_TYPE \
#--restore-with-alt-cpu=$ALT_CPU_TYPE
#--hmc-size=$HMC_SIZE \
#--command-line='earlyprintk=pl011,0x1c090000 console=ttyAMA0 lpj=19988480 norandmaps rw loglevel=8 mem=2G root=%(rootdev)s' \

