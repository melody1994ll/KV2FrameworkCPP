# KinectBaseLib

PROJECT(KinectBase)

#-------------------------------------------------------------
# find all files
#-------------------------------------------------------------
FILE(GLOB_RECURSE src "*.c*")
FILE(GLOB_RECURSE header "*.h*")

#-------------------------------------------------------------
# project and output
#-------------------------------------------------------------
ADD_LIBRARY(KinectBase SHARED ${src} ${header})
# output
SET_TARGET_PROPERTIES(KinectBase
					PROPERTIES
					RUNTIME_OUTPUT_DIRECTORY_DEBUG ${ROOT_BINARY_DIR}/bin
					LIBRARY_OUTPUT_DIRECTORY_DEBUG ${ROOT_BINARY_DIR}/bin
					ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${ROOT_BINARY_DIR}/bin
					RUNTIME_OUTPUT_DIRECTORY_RELEASE ${ROOT_BINARY_DIR}/bin
					LIBRARY_OUTPUT_DIRECTORY_RELEASE ${ROOT_BINARY_DIR}/bin
					ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${ROOT_BINARY_DIR}/bin
					)
# postfix
SET_TARGET_PROPERTIES(KinectBase
					PROPERTIES
					DEBUG_POSTFIX d)
					
#-------------------------------------------------------------
# include and lib
#-------------------------------------------------------------
TARGET_LINK_LIBRARIES(KinectBase ${OpenCV_LIBS})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(.)
INCLUDE_DIRECTORIES(${EIGEN_ROOT_DIR})

# kinect library and include
# INCLUDE_DIRECTORIES(${KINECT_INC})
# TARGET_LINK_LIBRARIES(KinectBase ${KINECT_LIB})
INCLUDE_DIRECTORIES("$ENV{KINECTSDK20_DIR}\\Inc")
TARGET_LINK_LIBRARIES(KinectBase "$ENV{KINECTSDK20_DIR}Lib\\x64\\kinect20.lib")