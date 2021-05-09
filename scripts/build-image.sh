#!/usr/bin/env bash

### build-image.sh -- build bootable disk image from docker image
###
### Usage:
###   build-image.sh [options] DOCKER_IMAGE DISK_FILE
###
### Parameters:
###   DOCKER_IMAGE             Name of docker image to use
###   DISK_FILE                File to write disk image to
###
### Options:
###   --help,     -h           Display this message.
###   --debug                  Print debug messages
###   --hostname, -H HOST      Hostname of disk image (default: "jackal")
###   --size,     -s SIZE      Size of disk image (see man truncate(1) for SIZE arg semantics)
###

function usage () {
    # Use file header as usage guide
    # Usage:
    #     usage
    # Reference: https://samizdat.dev/help-message-for-shell-scripts/
    sed -rn 's/^### ?/ /;T;p' "$0"
}

function log () {
    if [ -n "${debug-}" ]; then
        line_prefix="$0: Line ${BASH_LINENO[-2]}, Function ${FUNCNAME[-1]}() -->"
    else
        line_prefix="$0:"
    fi
    echo "$line_prefix $1" >&2
}

function cleanup () {
    if [ -n "${loopback_dev-}" ]; then
        log "Unmounting disk image device $loopback_dev..."
        sudo umount "$loopback_dev" && sync
        sudo losetup -d "$loopback_dev"
    fi

    if [ -d "${mount_dir-}" ]; then
        log "Removing mount dir $mount_dir..."
        rm -d "$mount_dir"
    fi

    if [ -n "${docker_container-}" ]; then
        log "Removing temporary docker container..."
        docker container rm "$docker_container" > /dev/null
    fi
}

function catch () {
    exit_code=$?
    cleanup
    exit $exit_code
}

function handle_image_dest () {
    if [ -z "${image_file-}" ]; then
        log "Error: No image filename provided"
        exit 1
    fi

    if [ ! -e "${image_name}" ]; then
        log "Info: Image file ${image_file} does not yet exist, creating"
        truncate -s "${image_file_size}" "${file_name}"
    fi

    if [ -b "${image_file}" ]; then
        log "Info: Image file ${image_file} is a block device, using physical disk methods"
        # TODO: create physical disk method here
        log "Error: not implemented"
        exit 1
    elif [ -f "${image_name}" ]; then
        log "Info: Image file ${image_file} is a regular file, using disk image methods"

        log "Info: Writing partition table to disk image"
        sfdisk "$file_name" >> /dev/null < partition-table.conf

        log "Info: Creating ext4 filesystem on disk image ${file_name}"
        mkfs.ext4 -o 1048576 "${file_name}"

        # TODO: add mount code, return generic mount dir that matches physical disk option
    else
        log "Error: Image file is neither a block device nor a regular file"
        exit 1
    fi

    # log "Info: Formatting disk image partition \"${loopback_dev}\" as ext4..."
    # mkfs.ext4 "${loopback_dev}" &> /dev/null
    
}

function main () {

    # if [ -f "$file_name" ]; then
    #     log "Existing image found, deleting..."
    #     rm "$file_name"
    # fi

    # log "Creating new disk image..."
    # truncate -s 5G "$file_name"

    # log "Writing partition table to disk image..."
    # sfdisk "$file_name" >> /dev/null < partition-table.conf

    # log "Attempting to bind loopback device to disk image..."
    # loopback_dev=$(sudo losetup -o 1048576 -f "$file_name" --show)
    # log "Success: got loopback device $loopback_dev"

    log "Formatting disk image partition as ext4..."
    # sudo mkfs.ext4 "$loopback_dev" &>> /dev/null

    log "Mounting new ext4 filesystem..."
    mkdir -p "$mount_dir"
    sudo mount -t ext4 "$loopback_dev" "$mount_dir"

    log "Copying filesystem from docker image to disk image..."
    docker_container=$(sudo docker run -d "$image_name" /bin/true)
    sudo docker export "$docker_container" \
        | pv -ptebars "$(docker image inspect "$image_name" | jq '.[0].Size')" \
        | sudo tar -xf - -X exclude.txt -C "$mount_dir"

    log "Writing system hostname \"$system_hostname\" to disk image..."
    echo "$system_hostname" | sudo tee "$mount_dir/etc/hostname" > /dev/null
    cat << EOF | sudo tee "$mount_dir/etc/hosts" > /dev/null
127.0.0.1	localhost
127.0.1.1	$system_hostname
EOF

    log "Installing extlinux bootloader on disk image..."
    sudo extlinux --install "$mount_dir"/boot 2>> /dev/null

    log "Writing syslinux mbr to disk image..."
    dd if=/usr/lib/syslinux/mbr/mbr.bin of="$file_name" bs=440 count=1 conv=notrunc 2>> /dev/null

    log "Success: disk image creation complete..."

}

# Exit on error
set -o errexit -o pipefail -o noclobber -o nounset
trap "catch" EXIT

# Parsing code reference: https://stackoverflow.com/q/192249#29754866
# -allow a command to fail with !’s side effect on errexit
# -use return value from ${PIPESTATUS[0]}, because ! hosed $?
# shellcheck disable=SC2251
! getopt --test > /dev/null
if [[ ${PIPESTATUS[0]} -ne 4 ]]; then
    log "Error: 'getopt --test' failed, cannot parse arguments"
    exit 1
fi

OPTIONS=hH:s:
LONGOPTS=help,debug,hostname:,size:

# -regarding ! and PIPESTATUS see above
# -temporarily store output to be able to check for errors
# -activate quoting/enhanced mode (e.g. by writing out “--options”)
# -pass arguments only via   -- "$@"   to separate them correctly
# shellcheck disable=SC2251
! PARSED=$(getopt --options=$OPTIONS --longoptions=$LONGOPTS --name "$0" -- "$@")
if [[ ${PIPESTATUS[0]} -ne 0 ]]; then
    # getopt has printed argument parsing error to stdout, exit
    exit 1
fi

# -read getopt’s output this way to handle the quoting right:
eval set -- "$PARSED"

# Set variable defaults
system_hostname="jackal"
mount_dir="tmp_mnt"
image_file_size="5G"

while true; do
    case "$1" in
        -h|--help)
            usage
            exit 1
            ;;
        --debug)
            debug=y
            shift
            ;;
        -H|--hostname)
            system_hostname="$2"
            shift 2
            ;;
        -s|--size)
            image_file_size="$2"
            shift 2
            ;;
        --)
            shift
            break
            ;;
        *)
            log "Error: Case statement doesn't match getopt for arg: $1, exiting"
            exit 1
            ;;
    esac
done

# Handle positional argument
if [[ $# -ne 2 ]]; then
    log "Error: exactly 2 positional arguments required, $# provided"
    exit 1
fi

image_name=$1
file_name=$2

log "Test: hello $image_name"

# main
