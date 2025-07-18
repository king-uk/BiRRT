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
```bash
cd BARN_dataset
python3 npy_to_txt.py
```

---
## Usage

```bash
cd RRT-Connect
mkdir builde && cd build
camke .. && make
./bi_rrt_only
```

## Result
<p align="center">
<img width="640" height="480" alt="Figure_1" src="https://github.com/user-attachments/assets/018c156d-0cfd-4b87-a194-97be7721a5d8" />
</p>
