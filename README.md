# Certifiable solver for real-time N-view triangulation

---
This repository contains the code 
for the fast N-view triangulation 
solver explained in [this paper](https://ieeexplore.ieee.org/document/10044919). 


**Authors:** 
[Mercedes Garcia-Salguero](https://mapir.isa.uma.es/mapirwebsite/?p=1718), 
[Javier Gonzalez-Jimenez](https://mapir.isa.uma.es/mapirwebsite/?p=1536)


**License:** [GPLv3](https://github.com/mergarsal/FastNViewTriangulation/blob/main/LICENSE)


If you use this code for your research, please cite:

```
@ARTICLE{,
    author = {Garcia-Salguero, Mercedes and Gonzalez-Jimenez, Javier},
     month = {{{{feb}}}},
     title = {Certifiable solver for real-time N-view triangulation},
   journal = {IEEE Robotics and Automation Letters},
      year = {2023},
      issn = {2377-3766},
       url = {http://mapir.isa.uma.es/papersrepo/2023/2023_mercedes_RAL_Nview_triangulation_paper.pdf},
       doi = {10.1109/LRA.2023.3245408}
}
```

## Dependencies

- Eigen 
- Ceres 

*Note*: Ceres is used in the example 
and test. You can remove that part 
and thus the dependency.



## Build

```
git clone https://github.com/mergarsal/FastNViewTriangulation.git
cd FastNViewTriangulation

mkdir build & cd build 

cmake .. 

make -jX

```

The compiled examples should be inside the `bin` directory. Run: 

```
        ./bin/example_base
```
 


## Install 
In `build` folder: 

```
        sudo make install
```

We also provide the uninstall script: 

```
        sudo make uninstall
```





# Use in external project 
1. Install our library with 

```
sudo make install 
```

2. In your project, find the library. 


```
find_package(NViewsTrian REQUIRED)
```

3. Include the library as target, e.g., 

```
add_executable(example_base ${CMAKE_CURRENT_SOURCE_DIR}/example_base.cpp)

target_link_libraries(example_base 
                              NViewsTrian      
                     )
```              

