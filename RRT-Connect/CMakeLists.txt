cmake_minimum_required(VERSION 3.5)
project(RRT_Connect)

find_package(PythonLibs REQUIRED)
find_package (Eigen3 REQUIRED NO_MODULE)
find_package(OpenMP REQUIRED)

include_directories(
    model/
    RRT/
    include/
    ${EIGEN3_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    include/EigenRand/
    include/matplotlibcpp
)

# 3. 소스 파일 명시
set(COMMON_SOURCES
    src/wmrobot/bi_rrt_only.cpp
    src/wmrobot/bi_rrt_mppi.cpp
    src/wmrobot/debug.cpp


    src/quadrotor/debug_1.cpp
)

foreach(SRC ${COMMON_SOURCES})
    get_filename_component(EXE_NAME ${SRC} NAME_WE)
    add_executable(${EXE_NAME} ${SRC})
    target_compile_options(${EXE_NAME} PRIVATE -O3)
    target_link_libraries(${EXE_NAME}     
        ${PYTHON_LIBRARIES}
        OpenMP::OpenMP_CXX
    )
endforeach()