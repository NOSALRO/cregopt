#!/bin/bash

# Save current directory path
cwd=$(pwd)

# Create a results directory
mkdir -p results/
cd results/

################
# Experiment 3 #
################

mkdir -p experiment_3/
cd experiment_3

## MinStance ##

mkdir -p min_stance/
cd min_stance

# CEM-MD #

mkdir -p cem_md/
cd cem_md

for i in $(seq 1 20); do
    mkdir -p "run_$i"/
    cd "run_$i"/
    rm -rf *

    "$cwd/../build/examples/exp3_min_stance_cem_md" $i

    cd ../
done

cd ../ # cd to min_stance/

# CrEGOpt #

mkdir -p cregopt/
cd cregopt

for i in $(seq 1 20); do
    mkdir -p "run_$i"/
    cd "run_$i"/
    rm -rf *

    "$cwd/../build/examples/exp3_min_stance_cregopt" $i

    cd ../
done

cd ../ # cd to min_stance/

cd ../ # cd to experiment_3/

## MinForce ##

mkdir -p min_force/
cd min_force

# CEM-MD #

mkdir -p cem_md/
cd cem_md

for i in $(seq 1 20); do
    mkdir -p "run_$i"/
    cd "run_$i"/
    rm -rf *

    "$cwd/../build/examples/exp3_min_force_cem_md" $i

    cd ../
done

cd ../ # cd to min_force/

# CrEGOpt #

mkdir -p cregopt/
cd cregopt

for i in $(seq 1 20); do
    mkdir -p "run_$i"/
    cd "run_$i"/
    rm -rf *

    "$cwd/../build/examples/exp3_min_force_cregopt" $i

    cd ../
done
