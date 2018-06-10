# poses_comparator

## ground_truth_estimator

### Build

```bash
$ mkdir build
$ cd build 
$ cmake ..
$ make
```

Do not forget include following lines in build/CMakeCache.txt

```cmake
//Flags used by the compiler during all build types.
CMAKE_CXX_FLAGS:STRING=-std=c++11
```

### Run

```bash
$ cd build
$ ./skViewer "route to dataset"
```





