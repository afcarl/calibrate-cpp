cmake_minimum_required(VERSION 2.8)
project(calibrate)


# packages
find_package(Boost COMPONENTS program_options system filesystem REQUIRED)
find_package(OpenCV REQUIRED)

#includes
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include_directories(${CMAKE_SOURCE_DIR}/include )

# binaries
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )

# sources
aux_source_directory(src/ SRC_LIST)

add_executable(${PROJECT_NAME} ${SRC_LIST})

# targets
target_link_libraries(${PROJECT_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${OpenCV_LIBS} )
