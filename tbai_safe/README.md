# tbai_safe 
[![Tests](https://github.com/Hier-Lab/tbai_safe/actions/workflows/python_test.yaml/badge.svg)](https://github.com/Hier-Lab/tbai_safe/actions/workflows/python_test.yaml)

## Installation

```bash
"${SHELL}" <(curl -L micro.mamba.pm/install.sh) # You might have to source your config again

# With GPU
conda env create -f .conda/tbai_safe.yaml
conda activate tbai_safe
pip3 install -e ".[all]"

# GPU-free
conda env create -f ./conda/tbai_safe-gpu-free.yaml
conda activate tbai_safe
pip3 install -e ".[all]"
```