from libcpp.vector cimport vector
from libcpp.utility cimport pair
from libcpp cimport float

cdef extern from "include/MovingLeastSquares.h":
    cdef cppclass MovingLeastSquares:
        void setParameters(float)
        pair[vector[vector[float]], vector[vector[float]]] apply(const vector[vector[float]]&)

cdef class PyMovingLeastSquares:
    cdef MovingLeastSquares _c_instance

    def set_parameters(self, float search_radius):
        self._c_instance.setParameters(search_radius)
        print("The search_radius is set to ", search_radius)

    def apply(self, point_cloud):
        cdef vector[vector[float]] point_cloud_cpp
        cdef vector[float] point_cpp
        for point in point_cloud:
            point_cloud_cpp.push_back(point)

        cdef pair[vector[vector[float]], vector[vector[float]]] result
        result = self._c_instance.apply(point_cloud_cpp)

        point_cloud_py = [[point[i] for i in range(point.size())] for point in result.first]
        normals_py = [[normal[i] for i in range(normal.size())] for normal in result.second]
        print("Moving least squares is done, returning ...")

        return point_cloud_py, normals_py

