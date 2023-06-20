#!/usr/bin/env bash

# Copyright (C) 2023 Paul Szenher
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

### rosdistro_merge -- merge multiple rosdistro repos into one
###
### Usage:
###   rosdistro_merge.sh [options] SOURCE_DIR[...]
###
### Parameters:
###   SOURCE_DIR[...]           Source directories to merge
###
### Options:
###   -h        --help          Display this message
###   -o dir    --output=dir    Name of merged output directory
###

function usage() {
    # Use file header as usage guide
    # Usage:
    #     usage
    # Reference: https://samizdat.dev/help-message-for-shell-scripts/

    sed -rn 's/^### ?/ /;T;p' "${0}"
}

# Set sane bash options and catch EXIT signal
set -o errexit -o pipefail -o noclobber -o nounset

# Check if system getopt is GNU enhanced version
if getopt --test >/dev/null; then
    logerror "\"getopt --test\" failed, this script requires GNU enhanced getopt"
    logerror "Cannot parse args, exiting"
    exit 1
fi

# Set getopt command-line options
OPTIONS=ho:
LONGOPTS=help,output:

# Parse arguments with getopt
PARSED=$(getopt --options="${OPTIONS}" --longoptions="${LONGOPTS}" --name "${0}" -- "${@}")

# Set positional arguments to getopt output
eval set -- "${PARSED}"

# Handle named arguments
while true; do
    case "${1}" in
	-h | --help)
	    usage
	    exit 1
	    ;;
	-o | --output)
	    output_dir="${2}"
	    shift 2
	    ;;
	--)
	    shift
	    break
	    ;;
	*)
	    >&2 echo "Internal Error:  Case statement doesn't match getopt for arg: ${1}, exiting"
	    exit 1
	    ;;
    esac
done

# Handle positional arguments
if [[ ${#} -lt 1 ]]; then
    >&2 echo "${0}: at least 1 positional argument(s) required, ${#} provided"
    usage
    exit 1
fi

# Handle output argument
if [ -z "${output_dir:-}" ]; then
    >&2 echo "${0}: must pass output directory, exiting..."
    usage
    exit 1
elif [ ! -e "${output_dir}" ]; then
    mkdir -p "${output_dir}"
elif [ -n "$(ls -A ${output_dir})" ]; then
    >&2 echo "${0}: selected output directory is not empty, exiting..."
    exit 1
elif [ ! -w "${output_dir}" ]; then
    >&2 echo "${0}: selected output directory is not writable by '${USER}', exiting..."
    exit 1
fi

source_dirs=( "${@}" )

function merge_yaml()
{
    local file_path source_dirs
    file_path="${1}"
    shift
    source_dirs=( "${@}" )
    
    >&2 echo "handling ${output_dir}/${file_path}..."
    mkdir -p "$(dirname ${output_dir}/${file_path})"
    
    printf "%s/${file_path}\n" "${source_dirs[@]}" \
	| while read input_file; do cat "${input_file}" 2>"/dev/null" \
					 || true; done \
 	| yq --yaml-output --slurp \
	     'reduce .[] as $item ({}; . * $item)' \
	     > "${output_dir}/${file_path}"
}

export -f merge_yaml
export output_dir

find "${source_dirs[@]}" \
     -type f \
     -name '*.yaml' \
     -not -path "*/releases/*.yaml" \
     -not -path "*/.github/*.yaml" \
     -printf '%P\n' \
    | sort \
    | uniq \
    | xargs -P $(nproc) -n 1 -I {} bash -c 'merge_yaml "$@"' _ {} "${source_dirs[@]}"
