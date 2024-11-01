cmake_minimum_required(VERSION 3.0)

# Setup the project.
# Its name is defined to be the controller directory name.
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT})

# Get C or C++ sources in the current directory (only).

set(SOURCES src/main.cpp src/NodeState.cpp )

include_directories(include)

# Set the  Webots home path (change it according to your installation method)
set(WEBOTS_HOME "/usr/local/webots")


#strategi compoenents
include_directories(/opt/offline/include/)
include_directories(/opt/offline/include/World
/opt/offline/include/tambahan)

find_package(Boost REQUIRED COMPONENTS thread program_options)


find_package(CGAL REQUIRED)
include_directories (${CGAL_INCLUDE_DIRS})
include_directories (${GMP_INCLUDE_DIRS})
include_directories (${MPFR_INCLUDE_DIRS})
set (cgal_LIBRARIES ${CGAL_LIBRARY} ${CGAL_Core_LIBRARY} ${GMP_LIBRARIES} ${MPFR_LIBRARIES})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -frounding-math")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frounding-math")



# Link with the Webots controller library.
link_directories($ENV{WEBOTS_HOME}/lib/controller)
set (LIBRARIES m ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
include_directories($ENV{WEBOTS_HOME}/include/controller/c $ENV{WEBOTS_HOME}/include/controller/cpp)

# Setup the target executable.
add_executable(${PROJECT} ${SOURCES})
target_link_libraries(${PROJECT}
RtDBcomm 
RtDBrtdb 
RtDButils
Boost::thread
${LIBRARIES} 
${cgal_LIBRARIES})

# Copy the target executable at the right location.
add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_BINARY_DIR}/${PROJECT} ${CMAKE_SOURCE_DIR}
)