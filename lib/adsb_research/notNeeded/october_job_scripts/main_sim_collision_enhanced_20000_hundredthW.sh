#!/bin/bash

#SBATCH --time=36:00:00   # walltime
#SBATCH --ntasks=1   # number of processor cores (i.e. tasks)
#SBATCH --nodes=1   # number of nodes
#SBATCH --mem-per-cpu=163840M   # memory per CPU core
#SBATCH -J "main_sim_collision_enhanced_20000_hundredthW"   # job name
#SBATCH --mail-user=jab128@byu.edu   # email address
#SBATCH --mail-type=BEGIN
#SBATCH --mail-type=END
#SBATCH --mail-type=FAIL


# Set the max number of threads to use for programs using OpenMP. Should be <= ppn. Does nothing if the program doesn't use OpenMP.
export OMP_NUM_THREADS=$SLURM_CPUS_ON_NODE

# LOAD MODULES, INSERT CODE, AND RUN YOUR PROGRAMS HERE
python3 main_sim_collision_enhanced.py > main_sim_collision_enhanced_20000_hundredthW.txt