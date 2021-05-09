#!/usr/bin/env bash

function log () {
    echo "$0 --> $1" >&2
}

function cleanup () {
    if [ -n "$loopback_dev" ]; then
        log "Unmounting disk image device $loopback_dev..."
        sudo umount "$loopback_dev" && sync
        sudo losetup -d "$loopback_dev"
    fi

    if [ -d "$mount_dir" ]; then
        log "Removing mount dir $mount_dir..."
        rm -d "$mount_dir"
    fi

    if [ -n "$docker_container" ]; then
        log "Removing temporary docker container..."
        docker container rm "$docker_container" > /dev/null
    fi
}

function catch () {
    exit_code=$?
    cleanup
    exit $exit_code
}

# Exit on error
set -o errexit -o pipefail -o noclobber -o nounset
trap "catch" EXIT

image_file="/tmp/linux_image.img"
mount_dir="tmp_mnt"
docker_image="jackal-kinetic"
jackal_hostname="jackal"

loopback_dev=""
docker_container=""

if [ -f $image_file ]; then
    log "Existing image found, deleting..."
    rm $image_file
fi

log "Creating new disk image..."
truncate -s 5G $image_file

log "Writing partition table to disk image..."
sfdisk $image_file >> /dev/null < partition-table.conf

log "Attempting to bind loopback device to disk image..."
loopback_dev=$(sudo losetup -o 1048576 -f $image_file --show)
log "Success: got loopback device $loopback_dev"

log "Formatting disk image partition as ext4..."
sudo mkfs.ext4 "$loopback_dev" &>> /dev/null

log "Mounting new ext4 filesystem..."
mkdir -p "$mount_dir"
sudo mount -t ext4 "$loopback_dev" "$mount_dir"

log "Copying filesystem from docker image to disk image..."
docker_container=$(sudo docker run -d "$docker_image" /bin/true)
sudo docker export "$docker_container" \
    | pv -ptebars "$(docker image inspect $docker_image | jq '.[0].Size')" \
    | sudo tar -xf - -X exclude.txt -C "$mount_dir"

log "Writing system hostname \"$jackal_hostname\" to disk image..."
echo $jackal_hostname | sudo tee "$mount_dir/etc/hostname" > /dev/null
cat << EOF | sudo tee "$mount_dir/etc/hosts" > /dev/null
127.0.0.1	localhost
127.0.1.1	$jackal_hostname
EOF

log "Installing extlinux bootloader on disk image..."
sudo extlinux --install "$mount_dir"/boot 2>> /dev/null

log "Writing syslinux mbr to disk image..."
dd if=/usr/lib/syslinux/mbr/mbr.bin of=$image_file bs=440 count=1 conv=notrunc 2>> /dev/null

log "Success: disk image creation complete..."
