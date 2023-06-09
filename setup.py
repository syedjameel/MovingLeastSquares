from setuptools import setup, Extension
from Cython.Build import cythonize, build_ext

cpp_library_name = "moving_least_squares"

# pcl_include_path = "/usr/local/include/pcl-1.13"
# eigen_include_path = "/usr/include/eigen3"
# vtk_include_path = "/usr/include/vtk-9.1"
# vtk_library_path = "/usr/local/lib"
# pcl_library_path = "/usr/local/lib"


pcl_include_path = "/usr/include/pcl-1.12"
eigen_include_path = "/usr/include/eigen3"
vtk_include_path = "/usr/local/include/vtk-9.2"
vtk_library_path = "/lib/x86_64-linux-gnu"
pcl_library_path = "/lib/x86_64-linux-gnu"

ext_modules = [
    Extension(
        "moving_least_squares",
        sources=["moving_least_squares.pyx", "src/MovingLeastSquares.cpp"],
        include_dirs=[pcl_include_path, eigen_include_path, vtk_include_path, "."],
        # libraries=[
        #     'vtkCommonCore-9.1', 'vtkCommonDataModel-9.1', 'vtkFiltersCore-9.1',
        #     'pcl_common', 'pcl_search', 'pcl_segmentation', 'pcl_visualization', 'pcl_surface', 'pcl_io',
        #     'pcl_registration', 'pcl_kdtree', 'pcl_octree', 'pcl_ml', 'pcl_features', 'pcl_sample_consensus',
        #     'boost_filesystem', 'boost_iostreams', 'pcl_io_ply'
        # ],
        libraries=[
            'vtkChartsCore-9.1', 'vtkCommonCore-9.1', 'vtkCommonDataModel-9.1', 'vtkFiltersCore-9.1',
            'pcl_common', 'pcl_io', 'pcl_search', 'pcl_segmentation', 'pcl_visualization', 'pcl_surface',
            'pcl_registration', 'boost_filesystem', 'boost_iostreams', 'pcl_io_ply', 'vtkIOGeometry-9.1',
            'vtkIOPLY-9.1', 'png16', 'usb-1.0', 'OpenNI2', 'OpenNI', 'pcap', 'vtkIOImage-9.1',
            'vtkIOLegacy-9.1', 'vtkImagingCore-9.1', 'vtkIOCore-9.1', 'z', 'bz2', 'lzma',
            'zstd', 'vtkFiltersHybrid-9.1', 'vtkRenderingCore-9.1', 'jsoncpp', 'dl', 'pthread',
            'jpeg', 'tinyxml', 'dbus-1', 'vtkDICOMParser-9.1', 'vtkmetaio-9.1', 'tiff', 'double-conversion',
            'vtkImagingSources-9.1', 'vtkFiltersGeometry-9.1', 'vtkCommonColor-9.1',
            'webp', 'jbig', 'deflate'
        ],
        library_dirs=[vtk_library_path, pcl_library_path],
        language="c++",
        extra_compile_args=["-std=c++14"],
    )
]

setup(
    ext_modules=cythonize(ext_modules, language_level=3),
    cmdclass={'build_ext': build_ext},
    py_modules=["moving_least_squares"]
)
