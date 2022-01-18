# Fast N-View triangulation

Coming soon ...

---
This repository contains the code 
for the fast N-view triangulation 
solver explained in [this paper](NONE) [1]. 


**Authors:** 
[Mercedes Garcia-Salguero](http://mapir.uma.es/mapirwebsite/index.php/people/290), 
[Javier Gonzalez-Jimenez](http://mapir.isa.uma.es/mapirwebsite/index.php/people/95-javier-gonzalez-jimenez)


**License:** [GPLv3](NONE)


If you use this code for your research, please cite:

```
NONE
```

## Dependencies

- Eigen 
- Ceres 

*Note*: Ceres is used in the example 
and test. You can remove that part 
and thus the dependency.



## Build
```
git clone https://github.com/mergarsal/FastMViewTriangulation.git
cd FastMViewTriangulation

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
`` 
sudo make install 
``

2. In your project, find the library. 
Add in your CMakeLists
``
add_definitions(-march=native)

find_package(NViewsTrian REQUIRED)
``

3. Include the library as target, e.g., 
``
add_executable(example_base ${CMAKE_CURRENT_SOURCE_DIR}/example_base.cpp)
target_link_libraries(example_base 
                              NViewsTrian      
                     )
``

