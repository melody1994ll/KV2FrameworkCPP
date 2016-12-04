#-------------------------------------------------------------------------------------------
# Set a Small Goal project
#-------------------------------------------------------------------------------------------

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(KV2Framework)

SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)


#---------------------------------------------------------------------
# Lib Names and Dirs
#---------------------------------------------------------------------
IF(WIN32)
	# postfix of dlls
	# SET(RUNTIME_OUTPUT_PATH ${project_binary_dir}/bin CACHE PATH "directory for dlls and binaries")
	# SET(EXECUTABLE_OUTPUT_PATH ${project_binary_dir}/bin CACHE PATH "directory for binaries")
	# SET(CMAKE_RUNTIOME_OUTPUT_DIRECTORY ${project_binary_dir}/bin)
	# SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${project_binary_dir}/bin)
	# SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${project_binary_dir}/lib)
ENDIF()

SET(ROOT_BINARY_DIR ${PROJECT_BINARY_DIR})
SET(EIGEN_ROOT_DIR ${CMAKE_SOURCE_DIR}/3rdparty/eigen3)
# set kinect library and include
IF(WIN32)
	SET(KINECT_INC "$(KINECTSDK20_DIR)\\Inc")
	SET(KINECT_LIB "$(KINECTSDK20_DIR)\\Lib\\$(Platform)\\kinect20.lib")
ENDIF()


#------------------------------------------------------------------------
# Find Dependencies
#------------------------------------------------------------------------

# add opencv
FIND_PACKAGE(OpenCV REQUIRED)
#INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})
#SET(REQUIRED_LIBRARIES ${OpenCV_LIBS})
IF(${OpenCV_VERSION} VERSION_LESS 2.3.1)
	MESSAGE(FATAL_ERROR "OpenCV version is not compatible: ${OpenCV_VERSION}")
ENDIF()

# add Eigen
#OPTION(USE_EIGEN3	"Set true to use Eigen in project")
#IF(USE_EIGEN3)
	#FIND_PACKAGE(Eigen3 REQUIRED)
	#SET(EIGEN3_INCLUDE_DIR "3rdparty/eigen3")
#ENDIF()
#INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

#--------------------------------------------------------------------------------
# DIRS
#--------------------------------------------------------------------------------
# ADD_SUBDIRECTORY(src src)
ADD_SUBDIRECTORY(sample sample)
ADD_SUBDIRECTORY(module module)




