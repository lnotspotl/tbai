# TBAI justfile

# Show available commands
help:
    @just --list

# Format C++ files in all directories (excluding dependencies and build)
format:
    #!/usr/bin/env bash
    folders=$(ls -d */ | grep -v dependencies | grep -v build)
    for folder in $folders; do
        for file in $(find $folder -name "*.hpp" -o -name "*.cpp"); do
            echo "[TBAI] Formatting $file"
            clang-format -i -style=file $file
        done
    done

# Generate conda environments for all pixi environments
pixi-generate-conda-envs:
    #!/usr/bin/env bash
    set -euo pipefail
    all_envs=$(pixi workspace environment list | grep -E '^- ' | cut -d':' -f1 | sed 's/^- //')
    for env in $all_envs; do
        pixi workspace export conda-environment -e $env > .conda/$env.yaml
    done

# Install the project (moved from pixi.toml)
install:
    cmake -B/tmp/cpmbuild -S. -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX && cmake --build /tmp/cpmbuild --parallel 8 && cmake --build /tmp/cpmbuild --target install

# Run tests (moved from pixi.toml)
test:
    cmake -Bbuild -S. -DTBAI_BUILD_TESTS=ON -DTBAI_BUILD_PYTHON=OFF -DTBAI_BUILD_DEPLOY_GO2=OFF && cd build && make -j8 && ctest --output-on-failure
