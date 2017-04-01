cmake_minimum_required(VERSION 2.8)

# Boost
set(Boost_LIBRARY_DIRS /usr/local/lib)
if(${PI_LIBRARY_TYPE} MATCHES "SHARED")
set(Boost_USE_STATIC_LIBS        OFF)
else()
set(Boost_USE_STATIC_LIBS        ON)
endif()
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME     OFF)
find_package(Boost REQUIRED COMPONENTS  filesystem system)
add_library(boost INTERFACE)
target_include_directories(boost INTERFACE ${Boost_INCLUDE_DIRS})

# Boost thread
if(WIN32)
  target_link_libraries(boost INTERFACE ${Boost_LIBRARIES} boost_thread-mt)
else()
  target_link_libraries(boost INTERFACE ${Boost_LIBRARIES} boost_thread)
endif()

# OpenCV
find_package(OpenCV REQUIRED PATHS)
add_library(opencv INTERFACE)
target_include_directories(opencv INTERFACE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(opencv INTERFACE ${OpenCV_LIBS})

# Eigen3
find_package(Eigen3 REQUIRED)
add_library(eigen INTERFACE)
target_include_directories(eigen INTERFACE ${EIGEN3_INCLUDE_DIR})

# Ceres
set(CERES_LIBS
    -lceres
    -lblas
    -llapack
    -lsuitesparseconfig
    -lcxsparse
    -lcholmod
    -lcamd
    -lamd
    -lcolamd)

add_library(ceres INTERFACE)
target_link_libraries(ceres INTERFACE ${CERES_LIBS})

# Glog
find_package(Glog REQUIRED)
add_library(glog INTERFACE)
target_include_directories(glog INTERFACE ${GLOG_INCLUDE_DIR})

target_link_libraries(glog INTERFACE ${GLOG_LIBRARY})
