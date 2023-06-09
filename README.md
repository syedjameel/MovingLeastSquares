
# PCL Moving Least Squares C++ and Python


## Description:
In this project we run MovingLeastSquare in C++ 
and also convert the MovingLeastSquares C++ code from the official pcl library 
to the Python module using Cython.


## Requirements
1. pcl-1.12
2. vtk-9.1
3. Python-3.10
4. C++14 or above

### To install the requirements
   ```shell
    sudo apt install libpcl-dev libvtk9-dev
  ```

### Note: Make sure that you have the pcl-1.12 version and vtk-9.1 version only for this project

### How to Run

1. Run the following command to navigate to the directory using:

   ```shell
   cd MovingLeastSquare/
   ```
2. Run the following command to install dependent libraries for python:

   ```shell
   pip3 install -r requirements.txt
   ```

3. Run the following commands to build the MovingLeastSquares in C++:
   ```shell
   mkdir build_cmake
   cd build_cmake/
   cmake ..
   make -j8
   ```
4. Now you can run the movingleastsquares binary from the build folder:
   ```shell
   ./movingleastsquares SEARCH_RADIUS PATH_TO_PCD_FILE
   ./movingleastsquares 10 ../sample_pcds/region_growing_tutorial.pcd
   ```
   or just use the following (default pcd file will be spring1.pcd)

   ```shell
   ./movingleastsquares
   ```

5. To convert the C++ code to Python module:
   ```shell
   cd MovingLeastSquare/
   python3 setup.py build_ext --inplace
   ```
6. Finally, run the following to use the converted C++ code in python:
   ```shell
   cd MovingLeastSquare/
   python3 test-moving-least-squares.py
   ```