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
    sed -rn 's/^### ?/ /;T;p' "${0}"
}

function ask () {
    # General-purpose y-or-n function
    # Reference: https://gist.github.com/davejamesmiller/1965569
    local prompt default reply
    if [[ ${2:-} = 'Y' ]]; then
        prompt='Y/n'
        default='Y'
    elif [[ ${2:-} = 'N' ]]; then
        prompt='y/N'
        default='N'
    else
        prompt='y/n'
        default=''
    fi
    while true; do
        # Ask the question (not using "read -p" as it uses stderr not stdout)
        echo -n "${1} [${prompt}] "
        # Read the answer (use /dev/tty in case stdin is redirected from somewhere else)
        read -r reply </dev/tty
        # Default?
        if [[ -z ${reply} ]]; then
            reply=${default}
        fi
        # Check if the reply is valid
        case "${reply}" in
            Y*|y*) return 0 ;;
            N*|n*) return 1 ;;
        esac
    done
}

function log () {
    if [ -n "${debug-}" ]; then
        line_prefix="${0}: Line ${BASH_LINENO[0]}, Function ${FUNCNAME[1]}() -->"
    else
        line_prefix="${0}:"
    fi
    echo "${line_prefix} ${1}" >&2
}

function cleanup () {
    if [ -n "${loopback_dev-}" ]; then
        log "Unmounting disk image device ${loopback_dev}..."
        sudo umount "${loopback_dev}" && sync
        sudo losetup -d "${loopback_dev}"
    fi

    if [ -d "${mount_dir-}" ]; then
        log "Removing mount dir ${mount_dir}..."
        rm -d "${mount_dir}"
    fi

    if [ -n "${docker_container-}" ]; then
        log "Removing temporary docker container..."
        docker container rm "${docker_container}"
    fi
}

function catch () {
    exit_code=${?}
    cleanup
    exit ${exit_code}
}

function init_disk_image () {
    # Initialize ${filename} disk image
    # Usage:
    #     init_disk_image ${filename} ${filesize}

    local numargs filename filesize
    numargs=2
    if [[ ${#} -ne ${numargs} ]]; then
        log "Error: ${FUNCNAME[0]}: exactly ${numargs} positional arguments required,"\
            "${#} provided"
        return 1
    fi
    filename=${1};filesize=${2}

    # If target image filename doesn't exist, create the file
    if [ ! -e "${filename}" ]; then
        log "Info: Image file ${filename} does not yet exist, creating"
        truncate -s "${filesize}" "${filename}"
    fi
   
}

function init_disk_partitions () {
    # Initialize ${filename} disk partitions
    # Usage:
    #     init_disk_partition ${filename}

    local numargs filename disk_model
    numargs=1
    if [[ ${#} -ne ${numargs} ]]; then
        log "Error: ${FUNCNAME[0]}: exactly ${numargs} positional arguments required,"\
            "${#} provided"
        return 1
    fi
    filename=${1}

    if [ -z "${filename-}" ]; then
        log "Error: No image filename provided"
        exit 1
    fi

    if [ -b "${filename}" ]; then
        log "Info: Image file ${filename} is a block device, using physical disk methods"

        disk_model=$(udevadm info "${filename}" -q property | sed -rn 's/^ID_MODEL=//;T;p')
        log "Warning: this action will erase ALL DATA on disk ${filename} (${disk_model})"
        if ! ask "Are you sure?" "N"; then
            log "Warning: disk partitioning cancelled, exiting"
            exit 1
        fi
        
        log "Info: Writing partition table to disk image"
        echo "2048 + L *" | sudo sfdisk -q -w always "${filename}"
    elif [ -f "${filename}" ]; then
        log "Info: Image file ${filename} is a regular file, using disk image methods"
        
        log "Info: Writing partition table to disk image"
        echo "2048 + L *" | sfdisk -q -w always "${filename}"
    else
        log "Error: Image file is neither a block device nor a regular file"
        exit 1
    fi
}

function init_system_hostname () {
    # Set hostname ${hostname} of filesystem at ${rootdir}
    # Usage:
    #     init_system_hostname ${hostname} ${rootdir}

    local numargs hostname rootdir
    numargs=2
    if [[ ${#} -ne ${numargs} ]]; then
        log "Error: ${FUNCNAME[0]}: exactly ${numargs} positional arguments required,"\
            "${#} provided"
        return 1
    fi
    hostname=${1}; rootdir=${2}

    echo "${hostname}" | sudo tee "${rootdir}/etc/hostname" > /dev/null
    cat << EOF | sudo tee "${rootdir}/etc/hosts" > /dev/null
127.0.0.1	localhost
127.0.1.1	${hostname}
EOF
}

function main () {
    log "Info: Initializing disk image"
    init_disk_image "${file_name}" "${image_file_size}"
    
    log "Info: Initializing disk partitions"
    init_disk_partitions "${file_name}"

    log "Info: Configuring loopback block device for disk image"
    loopback_dev=$(sudo losetup -o $((512 * 2048)) -f "${file_name}" --show)
    log "Info: Loopback device configured, \"${loopback_dev}\""

    if [ ! -e "${mount_dir}" ]; then
        mkdir -p "${mount_dir}"
    elif [ ! -d "${mount_dir}" ]; then
        log "Error: Target mount dir ${mount_dir} is not a directory"
        exit 1
    fi

    log "Info: Formatting disk partition as ext4"
    sudo mkfs.ext4 -q "${loopback_dev}"

    log "Info: Mounting formatted disk partition at ${mount_dir}"
    sudo mount -t ext4 "${loopback_dev}" "${mount_dir}"
    
    log "Copying filesystem from docker image to disk image..."
    docker_container=$(docker run -d "${image_name}" /bin/true)
    docker export "${docker_container}" \
        | pv -ptebars "$(docker image inspect "${image_name}" | jq '.[0].Size')" \
        | sudo tar -xf - -X exclude.txt -C "${mount_dir}"

    log "Writing system hostname \"${system_hostname}\" to disk image..."
    init_system_hostname "${system_hostname}" "${mount_dir}"
    
    log "Installing extlinux bootloader on disk image..."
    sudo extlinux --install "${mount_dir}"/boot

    log "Writing syslinux mbr to disk image..."
    sudo dd if=/usr/lib/syslinux/mbr/mbr.bin of="${file_name}" bs=440 count=1 conv=notrunc status=none 

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
! PARSED=$(getopt --options=${OPTIONS} --longoptions=${LONGOPTS} --name "${0}" -- "${@}")
if [[ ${PIPESTATUS[0]} -ne 0 ]]; then
    # getopt has printed argument parsing error to stdout, exit
    exit 1
fi

# -read getopt’s output this way to handle the quoting right:
eval set -- "${PARSED}"

# Set variable defaults
system_hostname="jackal"
mount_dir="tmp_mnt"
image_file_size="5G"

while true; do
    case "${1}" in
        -h|--help)
            usage
            exit 1
            ;;
        --debug)
            debug=y
            shift
            ;;
        -H|--hostname)
            system_hostname="${2}"
            shift 2
            ;;
        -s|--size)
            image_file_size="${2}"
            shift 2
            ;;
        --)
            shift
            break
            ;;
        *)
            log "Error: Case statement doesn't match getopt for arg: ${1}, exiting"
            exit 1
            ;;
    esac
done

# Handle positional argument
if [[ ${#} -ne 2 ]]; then
    log "Error: exactly 2 positional arguments required, ${#} provided"
    usage
    exit 1
fi

image_name=${1}
file_name=${2}

main
