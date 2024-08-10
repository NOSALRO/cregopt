#!/bin/bash

# Save current directory path
cwd=$(pwd)

mkdir -p results/
cd results/

################
# Experiment 2 #
################

mkdir -p experiment_2/
cd experiment_2

## Biped ##

mkdir -p biped/
cd biped

# CEM-MD #

mkdir -p cem_md/
cd cem_md

for i in $(seq 1 20); do
    mkdir -p "run_$i"/
    cd "run_$i"/
    rm -rf *

    "$cwd/../build/examples/exp2_biped_cem_md" $i

    cd ../
done

cd ../ # cd to biped/

# CrEGOpt #

mkdir -p cregopt/
cd cregopt

for i in $(seq 1 20); do
    mkdir -p "run_$i"/
    cd "run_$i"/
    rm -rf *

    "$cwd/../build/examples/exp2_biped_cregopt" $i

    cd ../
done

cd ../ # cd to biped/

cd ../ # cd to experiment_2/

## Quadruped ##

mkdir -p quad/
cd quad

# CEM-MD #

mkdir -p cem_md/
cd cem_md

for i in $(seq 1 20); do
    mkdir -p "run_$i"/
    cd "run_$i"/
    rm -rf *

    "$cwd/../build/examples/exp2_quad_cem_md" $i

    cd ../
done

cd ../ # cd to quad/

# CrEGOpt #

mkdir -p cregopt/
cd cregopt

for i in $(seq 1 20); do
    mkdir -p "run_$i"/
    cd "run_$i"/
    rm -rf *

    "$cwd/../build/examples/exp2_quad_cregopt" $i

    cd ../
done

cd ../ # cd to quad/

cd ../ # cd to experiment_2/

## Hexapod ##

mkdir -p hex/
cd hex

# CEM-MD #

mkdir -p cem_md/
cd cem_md

for i in $(seq 1 20); do
    mkdir -p "run_$i"/
    cd "run_$i"/
    rm -rf *

    "$cwd/../build/examples/exp2_hex_cem_md" $i

    cd ../
done

cd ../ # cd to hex/

# CrEGOpt #

mkdir -p cregopt/
cd cregopt

for i in $(seq 1 20); do
    mkdir -p "run_$i"/
    cd "run_$i"/
    rm -rf *

    "$cwd/../build/examples/exp2_hex_cregopt" $i

    cd ../
done
