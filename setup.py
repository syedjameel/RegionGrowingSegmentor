from setuptools import setup, Extension
from Cython.Build import cythonize, build_ext

import numpy as np

# Replace 'my_cpp_lib' with the actual name of your C++ library
cpp_library_name = "my_cpp_lib"

pcl_include_path = "/usr/local/include/pcl-1.13"
eigen_include_path = "/usr/include/eigen3"
vtk_include_path = "/usr/include/vtk-9.1"
vtk_library_path = "/usr/local/lib"
pcl_library_path = "/usr/local/lib"

ext_modules = [
    Extension(
        "region_growing_segmentor",
        sources=["region_growing_segmentor.pyx", "RegionGrowingSegmentor.cpp"],
        include_dirs=[pcl_include_path, eigen_include_path, vtk_include_path, "."],
        libraries=[
            'vtkCommonCore-9.1', 'vtkCommonDataModel-9.1', 'vtkFiltersCore-9.1',
            'pcl_common', 'pcl_search', 'pcl_segmentation', 'pcl_visualization'
        ],
        library_dirs=[vtk_library_path, pcl_library_path],
        language="c++",
        extra_compile_args=["-std=c++14"],
    )
]

setup(
    ext_modules=cythonize(ext_modules, language_level=3),
    cmdclass={'build_ext': build_ext},
    py_modules=["region_growing_segmentor"]
)
