# Feb 1 2021
# This script is used to write large numbers of job scripts for the super computer

mb_per_gb = 1024
models = ["main_sim_standard_model",
          "main_sim_message_enhanced",
          "main_sim_collision_enhanced",
          "main_sim_enhanced_message_and_collision"]
# models = ["main_sim_standard_model"]
vehicle_amounts = [# 1000,
                   # 1400,
                   # 2800,
                   5000,
                   # 8500,
                   10000,
                   # 14000,
                   # 20000,
                   ]
transmit_powers = ["hundredthW",
                   # "twentiethW",
                   "tenthW",
                   "oneW",
                   "tenW",
                   # "twentyW",
                   ]

# These values depend quite a bit on how much data you are saving off for figures
# These values are for sims with distances.append commented out.
memory_limit = {1000: 1,
                1400: 1,
                2800: 3,
                5000: 10,
                8500: 30,
                10000: 32,
                14000: 80,
                20000: 256}

time_limit = {1000: "00:30:00",
              1400: "00:30:00",
              2800: "01:00:00",
              5000: "05:00:00",
              8500: "08:00:00",
              10000: "12:00:00",
              14000: "20:00:00",
              20000: "48:00:00"}

for x in models:
    for y in vehicle_amounts:
        for z in transmit_powers:
            job_name = str(x) + "_" + str(y) + "_" + str(z)
            f = open(job_name + ".sh", "w")
            f.write("#!/bin/bash\n\n")
            f.write("#SBATCH --time=" + time_limit[y] + "   # walltime\n")
            f.write("#SBATCH --ntasks=1   # number of processor cores (i.e. tasks)\n")
            f.write("#SBATCH --nodes=1   # number of nodes\n")
            f.write("#SBATCH --mem-per-cpu=" + str(mb_per_gb * memory_limit[y]) + "M" + "   # memory per CPU core\n")
            f.write("#SBATCH -J \"" + job_name + "\"   # job name\n")
            f.write("#SBATCH --mail-user=barrettjonathan128@gmail.com   # email address\n")
            f.write("#SBATCH --mail-type=BEGIN\n")
            f.write("#SBATCH --mail-type=END\n")
            f.write("#SBATCH --mail-type=FAIL\n\n\n")
            f.write("# Set the max number of threads to use for programs using OpenMP. Should be <= ppn. Does nothing if the program doesn't use OpenMP.\n")
            f.write("export OMP_NUM_THREADS=$SLURM_CPUS_ON_NODE\n\n")
            f.write("# LOAD MODULES, INSERT CODE, AND RUN YOUR PROGRAMS HERE\n")
            f.write("python3 " + str(x) + ".py > " + job_name + ".txt")
            f.close()
