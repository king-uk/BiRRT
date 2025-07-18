# BiRRT

Dependancy
EigenRand needs to be named as same as in CMakeLists.txt.

Eigen 3.3.9
EigenRand
OpenMP
Map Download and Modification
git clone https://github.com/i-ASL/BiC-MPPI.git
cd BiC-MPPI
Please download BARN_dataset/grid_files from BARN_dataset

cd BARN_dataset
python3 npy_to_txt.py
barn_mod Map modification in the BARN dataset with extended boundaries and inflated obstacles

Usage
Point-mass quadrotor landing problem
cd BiC-MPPI
mkdir build && cd build
cmake.. -Dquadrotor=1 && make
./bi_mppi   # or other variants (mppi, log_mppi, cluster_mppi)
