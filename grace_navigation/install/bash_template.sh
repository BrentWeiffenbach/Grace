#!/bin/bash

verbose=false

function usage() {
    echo "Usage:"
    echo "-v: Enable verbose logging"
    echo "-h: Display this help message"
}

while getopts :vh flag
do
    case "${flag}" in
        v) verbose=true;;
        h) usage;;
        *) ;;
    esac
done

grace_dir="$(cd "$(dirname "$0")" && pwd)"
# Make the script run the same in install or in just Grace
if [[ "$grace_dir" == */install ]]; then
    grace_dir="$(dirname "$grace_dir")"
fi

cd "$grace_dir" || exit

# Script code here

cd "$grace_dir" || exit
exit 0