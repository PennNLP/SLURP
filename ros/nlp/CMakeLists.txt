##############################################################################
# CMake
##############################################################################

# set(CMAKE_VERBOSE_MAKEFILE TRUE)

##############################################################################
# Catkin
##############################################################################

project(nlp)

# Some other arguments to this function:
#   INCLUDE_DIRS: export include directories (typically 'INCLUDE_DIRS include')
#   LIBRARIES   : export cpp libraries (used to create pkg-config and cmake-config modules)
#   CFG_EXTRAS  : export cmake modules (e.g.'CFG_EXTRAS my-module.cmake')
#   DEPENDS     : 
catkin_project(nlp
    DEPENDS std_msgs rospy 
)

# The following generates ROS_INCLUDE_DIRS and ROS_LIBRARIES for use with cpp targets
# Move this call to stack.xml if you are calling find_package on a particular
# component alot.
find_package(ROS REQUIRED COMPONENTS std_msgs rospy )
include_directories(${ROS_INCLUDE_DIRS})

##############################################################################
# Binaries
##############################################################################

# An example executable
#add_executable(nlp main.cpp)
#target_link_libraries(nlp ${ROS_LIBRARIES})
#install(TARGETS nlp RUNTIME DESTINATION bin)

# An example library
#add_library(nlp mylib.cpp)
#target_link_libraries(nlp ${ROS_LIBRARIES})
#install(TARGETS nlp DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
