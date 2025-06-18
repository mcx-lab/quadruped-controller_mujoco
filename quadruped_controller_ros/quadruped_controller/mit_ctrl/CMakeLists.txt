include_directories("./")
include_directories("${PROJECT_SOURCE_DIR}/common/include/")
include_directories("${PROJECT_SOURCE_DIR}/third-party/")
include_directories("${PROJECT_SOURCE_DIR}")
include_directories(${CMAKE_BINARY_DIR})

add_subdirectory(convexMPC)
add_subdirectory(WBC)
add_subdirectory(WBC_Ctrl)