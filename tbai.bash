#!/usr/bin/env bash

function print_help() {
	echo "Usage: ./tbai2.bash [--format]"
}

function format() {
    folders=$(ls -d */| grep -v dependencies | grep -v build)
    for folder in $folders; do
        for file in $(find $folder -name "*.hpp" -o -name "*.cpp"); do
            echo "[TBAI] Formatting $file"
            clang-format -i -style=file $file
        done
    done
}

if [[ "$1" == "--help" || "$1" == "-h" || -z "$1" ]]; then
	print_help
	exit
fi

if [[ "$1" == "--format" ]]; then
    format
    exit
fi

print_help
