#!/bin/bash

# Run gen_terrain to generate thw step terrain used on scenario 3 #
cd ../build/
./gen_terrain
cd ../tools/

# Save current directory path
cwd=$(pwd)

mkdir -p results/
cd results/

################
# Experiment 1 #
################

mkdir -p experiment_1/
cd experiment_1

## Scenario 1 ##

mkdir -p scenario_1/
cd scenario_1

# CrEGOpt #

mkdir -p cregopt/
cd cregopt

for i in $(seq 1 20); do
    mkdir -p "run_$i"/
    cd "run_$i"/
    rm -rf *

    "$cwd/../build/examples/exp1_scen1_cregopt" $i

    cd ../
done

cd ../ # cd to scenario_1/

# Towr #

mkdir -p towr/
cd towr

for i in $(seq 1 20); do
    mkdir -p "run_$i"/
    cd "run_$i"/
    rm -rf *

    "$cwd/../build/examples/exp1_scen1_towr" $i

    cd ../
done

cd ../ # cd to scenario_1/

cd ../ # cd to experiment_1/

## Scenario 2 ##

mkdir -p scenario_2/
cd scenario_2

# CrEGOpt #

mkdir -p cregopt/
cd cregopt

for i in $(seq 1 20); do
    mkdir -p "run_$i"/
    cd "run_$i"/
    rm -rf *

    "$cwd/../build/examples/exp1_scen2_cregopt" $i

    cd ../
done

cd ../ # cd to scenario_2/

# Towr #

mkdir -p towr/
cd towr

for i in $(seq 1 20); do
    mkdir -p "run_$i"/
    cd "run_$i"/
    rm -rf *

    "$cwd/../build/examples/exp1_scen2_towr" $i

    cd ../
done

cd ../ # cd to scenario_2/

cd ../ # cd to experiment_1/

## Scenario 3 ##

mkdir -p scenario_3/
cd scenario_3

# CrEGOpt #

mkdir -p cregopt/
cd cregopt

for i in $(seq 1 20); do
    mkdir -p "run_$i"/
    cd "run_$i"/
    rm -rf *

    "$cwd/../build/examples/exp1_scen3_cregopt" $i

    cd ../
done

cd ../ # cd to scenario_3/

# Towr #

mkdir -p towr/
cd towr

for i in $(seq 1 20); do
    mkdir -p "run_$i"/
    cd "run_$i"/
    rm -rf *

    "$cwd/../build/examples/exp1_scen3_towr" $i

    cd ../
done
