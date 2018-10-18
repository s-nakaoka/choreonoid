#!/bin/bash

files=(`ls | sed s/\.wrl//g`)

for file in ${files[@]}; do
    mkdir -p urdf/${file}
    simtrans -i ${file}.wrl -o urdf/${file}/${file}.urdf
done;
