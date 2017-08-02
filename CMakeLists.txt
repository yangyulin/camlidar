CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(camlidar)

SET(CMAKE_MODULE_PATH  ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(G2O REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(X11 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization system filesystem thread program_options date_time regex timer chrono)

# Try to compile with c++11
# http://stackoverflow.com/a/25836953
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

file(GLOB_RECURSE thirdparty_sources thirdparty/yaml-cpp/*.cpp)

INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR}
                    ${CSPARSE_INCLUDE_DIR}
                    ${OpenCV_INCLUDE_DIRS}
                    ${G2O_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${X11_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${GTSAM_INCLUDE_DIR}
        	        ${PCL_INCLUDE_DIRS}
                    thirdparty
				    src
                    include)

ADD_EXECUTABLE( imgrect_zjudata
                src/imgrect_zjudata.cpp )

TARGET_LINK_LIBRARIES(  imgrect_zjudata
                        ${OpenCV_LIBS}
                        ${G2O_LIBRARIES}
                        ${Boost_LIBRARIES}
                        csparse
                        cxsparse)

ADD_EXECUTABLE( imgrect_vi
                src/imgrect_vi.cpp)

TARGET_LINK_LIBRARIES(  imgrect_vi
                        ${OpenCV_LIBS}
                        ${G2O_LIBRARIES}
                        ${Boost_LIBRARIES}
                        csparse
                        cxsparse)

ADD_EXECUTABLE( imgrect_3p
                src/imgrect_3p.cpp)

TARGET_LINK_LIBRARIES(  imgrect_3p
                        ${OpenCV_LIBS}
                        ${G2O_LIBRARIES}
                        ${Boost_LIBRARIES}
                        csparse
                        cxsparse)
