# get the head image
PROJECT(HeadDetect)

#-------------------------------------------------------------
# find all files
#-------------------------------------------------------------
FILE(GLOB src "*.c*")
# FILE(GLOB header "*.h*")

#-------------------------------------------------------------
# project and output
#-------------------------------------------------------------
ADD_EXECUTABLE(HeadDetect ${src})
# output
SET_TARGET_PROPERTIES(HeadDetect
					PROPERTIES
					RUNTIME_OUTPUT_DIRECTORY_DEBUG ${ROOT_BINARY_DIR}/bin
					LIBRARY_OUTPUT_DIRECTORY_DEBUG ${ROOT_BINARY_DIR}/bin
					ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${ROOT_BINARY_DIR}/bin
					RUNTIME_OUTPUT_DIRECTORY_RELEASE ${ROOT_BINARY_DIR}/bin
					LIBRARY_OUTPUT_DIRECTORY_RELEASE ${ROOT_BINARY_DIR}/bin
					ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${ROOT_BINARY_DIR}/bin
					)

#-------------------------------------------------------------
# include and lib
#-------------------------------------------------------------
TARGET_LINK_LIBRARIES(HeadDetect ${OpenCV_LIBS})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(${KinectBase_SOURCE_DIR})
TARGET_LINK_LIBRARIES(HeadDetect KinectBase)
MESSAGE(STATUS ${KinectBase_SOURCE_DIR})
MESSAGE(STATUS ${KinectBase})

# kinect library and include
# INCLUDE_DIRECTORIES(${KINECT_INC})
# TARGET_LINK_LIBRARIES(HeadDetect ${KINECT_LIB})
INCLUDE_DIRECTORIES("$ENV{KINECTSDK20_DIR}\\Inc")
TARGET_LINK_LIBRARIES(HeadDetect "$ENV{KINECTSDK20_DIR}Lib\\x64\\kinect20.lib")

INCLUDE_DIRECTORIES(${EIGEN_ROOT_DIR})
