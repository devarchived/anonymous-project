#!/bin/bash

# Configurable parameters
simulation_file="scratch/wifi-eht-static-scenario/wifi-eht-static-scenario.cc"
printOutput=true
poissonLambda=500

# Function to run simulations for a given numBss
run_simulations() {
    local numBss=$1
    for seed in {1..2}; do
        echo "Running simulation with file=$simulation_file, numBss=$numBss, seed=$seed"
        ./ns3 run "$simulation_file" -- --seed=$seed --numBss=$numBss --poissonLambda=$poissonLambda --printOutput=$printOutput
    done
}

# Export the function so it can be used by parallel processes
export -f run_simulations

./ns3 clean
./ns3 configure --disable-werror --enable-examples
./ns3 build

MAX_PARALLEL=1

# Run simulations in parallel for numBss from 1 to 4
for numBss in {1..4}; do
    # Run all seed values for the current numBss in parallel
    for seed in {1..50}; do
        echo "Running simulation with file=$simulation_file, numBss=$numBss, seed=$seed"
        ./ns3 run "$simulation_file" -- --seed=$seed --numBss=$numBss --poissonLambda=$poissonLambda --printOutput=$printOutput &

        if [[ $(jobs -r -p | wc -l) -ge $MAX_PARALLEL ]]; then
        wait -n
        fi
    done
    # Wait for all background jobs for the current numBss to finish
    wait
    echo "All simulations for numBss=$numBss completed."
done

echo "All simulations completed."