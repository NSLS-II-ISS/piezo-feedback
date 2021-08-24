#!/bin/bash
PATH=/opt/conda/bin:$PATH
#ENV_NAME="collection-17Q3.0"
ENV_NAME="collection-2021-1.2"
source activate $ENV_NAME

python3 /epics/iocs/piezo-feedback/piezo-feedback/piezo_fb.py
