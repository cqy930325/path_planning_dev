cmake_minimum_required(VERSION 2.8)
project(PATH_PLANNING_DEV)

set(CMAKE_C_FLAGS "-std=c99")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -std=c++11")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

# Set library and headers.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

### 3rd party ###

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# LIBUSB
find_package(PkgConfig)
pkg_check_modules(LIBUSB libusb-1.0)

# Boost
set(Boost_LIBRARY_DIRS /usr/local/lib)
#if(${PI_LIBRARY_TYPE} MATCHES "SHARED")
#set(Boost_USE_STATIC_LIBS        OFF)
#else()
set(Boost_USE_STATIC_LIBS        ON)
#endif()
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

set(EXT_LIBS
    ${OpenCV_LIBS}
#    ${Boost_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${BoostThreadLib}
    ${LIBUSB_LIBRARIES}
#    ${GLOG_LIBRARY}
    ${Eigen3_LIBS}
#    ${CERES_LIBS}
    -lpthread
    -lblas
    -llapack
    boost
    glog
    eigen
    ceres
    )

set(PI_LIBS
    -lpi_core_config
    -lpi_core_marker
    -lpi_core_model
    -lpi_core_planning
    -lpi_core_sensor
    -lpi_core_util
    -lpi_interface
    -lpi_core_config
    -lpi_core_slam
    -luvc)
# Build the sample apps.
set(APPS
    "apps/v1_viewer.cpp"
    "apps/vision_controller.cpp"
    "apps/cpp_unit_test.cpp"
    "apps/map_help.cpp"
        )
add_library (CPP_LIB "apps/coverage_path_planning.cpp")
target_include_directories(CPP_LIB PUBLIC "apps")

foreach(app ${APPS})
    get_filename_component(EXE_NAME ${app} NAME_WE)
    add_executable(${EXE_NAME} ${app})
    target_link_libraries(${EXE_NAME} ${EXT_LIBS} ${PI_LIBS} CPP_LIB)
endforeach(app)
