#!/bin/bash

# Configurable parameters
simulation_file="scratch/wifi-eht-roaming-scenario/wifi-eht-roaming-scenario.cc"
printOutput=true
poissonLambda=500
wallLoss=5

# Function to run simulations for a given numBss
run_simulations() {
    local maxMissedBeacons=$1
    for seed in {1..2}; do
        echo "Running simulation with file=$simulation_file, maxMissedBeacons=$maxMissedBeacons, seed=$seed"
        ./ns3 run "$simulation_file" -- --seed=$seed --maxMissedBeacons=$maxMissedBeacons --poissonLambda=$poissonLambda --wallLoss=$wallLoss --printOutput=$printOutput
    done
}

# Export the function so it can be used by parallel processes
export -f run_simulations

# ./ns3 clean
# ./ns3 configure --disable-werror --enable-examples --build-profile=optimized
# ./ns3 build

MAX_PARALLEL=7

# Run simulations in parallel for numBss from 1 to 4
for maxMissedBeacons in {1..10}; do
    # Run all seed values for the current numBss in parallel
    for seed in {1..50}; do
        echo "Running simulation with file=$simulation_file, maxMissedBeacons=$maxMissedBeacons, seed=$seed"
        ./ns3 run "$simulation_file" -- --seed=$seed --maxMissedBeacons=$maxMissedBeacons --poissonLambda=$poissonLambda --wallLoss=$wallLoss --printOutput=$printOutput &

        if [[ $(jobs -r -p | wc -l) -ge $MAX_PARALLEL ]]; then
        wait -n
        fi
    done
    # Wait for all background jobs for the current numBss to finish
    wait
    echo "All simulations for maxMissedBeacons=$maxMissedBeacons completed."
done

echo "All simulations completed."
