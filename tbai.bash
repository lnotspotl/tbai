#!/usr/bin/env bash

function print_help() {
	echo "Usage: ./tbai [--format|--lint|--build|--test|--docs|--rebuild_docs]"
}

function lint() {
    folders=$(ls -d */| grep -v dependencies)
    for folder in $folders; do
        cpplint --recursive $folder
    done
}

function format() {
    folders=$(ls -d */| grep -v dependencies)
    for folder in $folders; do
        for file in $(find $folder -name "*.hpp" -o -name "*.cpp"); do
            echo "[TBAI] Formatting $file"
            clang-format -i -style=file $file
        done
    done
}

function build() {
    folders=$(ls -d */| grep -v dependencies)
    ros_packages=""
    # in each folder check whether package.xml and CMakeLists.txt exist
    for folder in $folders; do
        if [[ -f $folder/CMakeLists.txt ]] && [[ -f $folder/package.xml ]]; then
            package=$(basename $folder)
            ros_packages+=" $package"
        fi
    done
    echo "[TBAI] Building ROS packages: $ros_packages"
    catkin build $ros_packages
}

function test() {
    folders=$(ls -d */| grep -v dependencies)
    ros_packages=""
    # for each folder, check where test folder exists
    for folder in $folders; do
        if [[ -d $folder/test ]]; then
            echo "[TBAI] Running tests in $folder"
            package=$(basename $folder)
            ros_packages+=" $package"
        fi
    done
    echo "[TBAI] Running tests for ROS packages: $ros_packages"
    catkin test $ros_packages
}

function open_docs() {
    script_dir=$(dirname "$0")
    docs_dir=${script_dir}/../../build/tbai_docs/output/doxygen/html/index.html
    google-chrome $docs_dir
}


if [[ "$1" == "--help" || "$1" == "-h" || -z "$1" ]]; then
	print_help
	exit
fi

if [[ "$1" == "--lint" ]]; then
	lint
	exit
fi

if [[ "$1" == "--format" ]]; then
    format
    exit
fi

if [[ "$1" == "--build" ]]; then
    build
    exit
fi

if [[ "$1" == "--test" ]]; then
    test
    exit
fi

if [[ "$1" == "--docs" ]]; then
    open_docs
    exit
fi

if [[ "$1" == "--rebuild_docs" ]]; then
    catkin clean tbai_docs
    catkin build tbai_docs
fi

print_help