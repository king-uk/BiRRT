# BiRRT

## Dependancy

 - Eigen 3.3.9
 - EigenRand
 - OpenMP

## Map Download and Modification

```bash
git clone https://github.com/king-uk/BiRRT.git
cd RRT-Connect
```

## Please download BARN_dataset/grid_files from [BARN_dataset](https://www.cs.utexas.edu/~xiao/BARN/BARN.html)
cd BARN_dataset
python3 npy_to_txt.py


---
## Usage

```bash
cd RRT-Connect
mkdir builde && cd build
camke .. && make
./bi_rrt_only
```
