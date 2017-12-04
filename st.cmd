#!/bin/bash
PATH=$PATH:/opt/conda/bin
ENV_NAME="collection-17Q3.0"
source activate $ENV_NAME

python /epics/iocs/piezo-feedback/piezo_fb.py
