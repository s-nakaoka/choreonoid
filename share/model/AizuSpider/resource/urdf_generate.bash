#!/bin/bash

files=("CHASSIS" "FLIPPER" "LEDJ1-W" "MAIN_BELT" "MCM-4350FISH" "SUB_BELT" "UTM-30LN")

for file in ${files[@]}; do
    mkdir -p urdf/${file}
    simtrans -i ${file}.wrl -o urdf/${file}/${file}.urdf
done;
            
