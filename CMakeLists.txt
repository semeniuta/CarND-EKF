project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set (core_sources src/tools.cpp src/FusionEKF.cpp src/kalman_filter.cpp src/tools.h src/FusionEKF.h src/kalman_filter.h)

    if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

    include_directories(/usr/local/include)
    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/lib)
    link_directories(/usr/local/opt/openssl/lib)

    file(GLOB LIBUV_DIR /usr/local/Cellar/libuv/1*/lib)
    link_directories(${LIBUV_DIR})

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(ExtendedKF src/main.cpp ${core_sources})
target_link_libraries(ExtendedKF z ssl uv uWS)

add_executable(validate src/validate.cpp ${core_sources})
target_link_libraries(validate z ssl uv uWS)
