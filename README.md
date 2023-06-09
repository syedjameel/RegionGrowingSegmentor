
# PCL Region Growing Segmentor Algorithm from C++ to Python


## Description:
In this project we convert the RegionGrowingSegmentor Algorithm from C++ code from the official pcl library
to the Python module using Cython.


## Requirements
1. pcl-1.13
2. vtk-9.1
3. Python-3.10
4. C++14 or above

### To install the vtk
   ```shell
    sudo apt install libvtk9-dev
  ```

### Note: 
1. Make sure that you have the pcl-1.13 version and vtk-9.1 version only for this project
2. Currently, you need to install pcl-1.13 from source because its not available as a pip package currently

### How to Run

1. Run the following command to navigate to the directory using:

   ```shell
   cd RegionGrowingSegmentor/
   ```
2. Run the following command to install dependent libraries for python:

   ```shell
   pip3 install -r requirements.txt
   ```

3. To convert the C++ code to Python module:
   ```shell
   cd RegionGrowingSegmentor/
   python3 setup.py build_ext --inplace
   ```
4. Finally, run the following to use the converted C++ code in python:
   ```shell
   cd RegionGrowingSegmentor/
   python3 test-this-seg.py
   ```