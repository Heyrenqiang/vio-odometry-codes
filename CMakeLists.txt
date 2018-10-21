cmake_minimum_required(VERSION 2.8)

project(sfm-project)
set(CMAKE_CXX_FLAGS "-std=c++11")

set(CMAKE_BUILD_TYPE "Debug")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})

include_directories(${Sophus_INCLUDE_DIRS})

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(vioodometry
    src/main.cpp
    src/frame.cpp
    src/featurepoints.cpp
    src/feature_track.cpp
    src/datareading.cpp
    src/parameters.cpp
    src/vioodometry.cpp
    src/cameramodel.cpp
    src/solvepose.cpp
    )

target_link_libraries(vioodometry ${OpenCV_LIBS} ${CERES_LIBRARIES}
    ${Sophus_LIBRARIES}
    ${Pangolin_LIBRARIES})
