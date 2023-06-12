#!/usr/bin/env bash

set -e

qemu_bin="qemu-system-x86_64"

qemu_machine_type="q35"
qemu_cpu_type="Haswell,kvm=off"

qemu_hdd_file="/dev/shm/focal.img"

${qemu_bin} -nographic \
	    -nodefaults \
	    -runas nobody \
	    -name "JackalVelodyneVM" \
	    -enable-kvm \
	    -machine ${qemu_machine_type} \
	    -cpu     ${qemu_cpu_type}     \
	    -smp sockets=1,cpus=4,cores=2 \
	    -m 2G \
	    -drive file="${qemu_hdd_file}" \
	    -serial mon:stdio \
	    \
	    -nic user,model=e1000 \
	    -nic socket,mcast=239.255.1.1:2368
