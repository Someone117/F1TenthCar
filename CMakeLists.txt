cmake_minimum_required(VERSION 3.7)
project(RACECAR)

set(CMAKE_CXX_STANDARD 20)
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -D_REENTRANT -lpthread")
endif()

set(SOURCE_DIR "./")
file(GLOB_RECURSE SOURCES "${SOURCE_DIR}/src/*.cpp" "${SOURCE_DIR}/src/*.hpp" "${SOURCE_DIR}/src/*.h" "${SOURCE_DIR}/src/*.c")

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} -static)
