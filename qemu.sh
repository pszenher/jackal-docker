#!/usr/bin/env bash

qemu_bin="qemu-system-x86_64"

qemu_machine_type="q35"
qemu_cpu_type="Haswell,kvm=off"

qemu_hdd_file="/dev/shm/test.img"

${qemu_bin} -nographic \
	    -nodefaults \
	    -runas nobody \
	    -name "JackalVM" \
	    -enable-kvm \
	    -machine ${qemu_machine_type} \
	    -cpu     ${qemu_cpu_type}     \
	    -smp sockets=1,cpus=4,cores=2 \
	    -m 2G \
	    -drive file="${qemu_hdd_file}" \
	    -serial mon:stdio \
	    \
	    -device pcie-pci-bridge,id=br1,bus=pcie.0,addr=1 \
	    -device pcie-pci-bridge,id=br2,bus=pcie.0,addr=2 \
	    -device pcie-pci-bridge,id=br3,bus=pcie.0,addr=3 \
	    \
	    -device e1000,id=nic0,netdev=net0,bus=br1,addr=0,acpi-index=1,mac="c4:00:ad:07:f5:bb" \
	    -device e1000,id=nic1,netdev=net1,bus=br3,addr=0,acpi-index=0,mac="c4:00:ad:07:f5:bc" \
	    \
	    -netdev socket,id=net0,mcast=239.255.1.1:2368 \
	    -netdev user,id=net1,net=192.168.0.0/24,dhcpstart=192.168.0.100,hostfwd=tcp::8765-:8765 \
	    \
	    -usb \
	    -chardev file,id=jackal,path=/tmp/jackaldev.log \
	    -device usb-serial,chardev=jackal

# -device pxb-pcie,id=pcie.3,bus_nr=3
# -device pcie-root-port,id=rp30,bus=pcie.3,chassis=0,slot=1,addr=3.0

# -netdev socket,id=net0,udp=127.0.0.1:2368,localaddr=127.0.0.1:23680 \
