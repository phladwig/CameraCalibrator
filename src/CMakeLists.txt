#   CmakeLists for CameraCalibrator

include_directories(${CMAKE_BINARY_DIR}/src ${OpenCV_INCLUDE_DIRS} ${VIDEOINPUT_INCLUDE_DIR} ${REALSENSE_SDK_INCLUDE_DIR})
link_directories(${CMAKE_BINARY_DIR}/lib "${CMAKE_BINARY_DIR}/lib" ${VIDEOINPUT_LIBRARIES_DIR} ${REALSENSE_SDK_LIBRARIES_DIR})
file(GLOB_RECURSE CAMERACALIBRATOR_HEADER "${CMAKE_CURRENT_SRC_DIR}" *.h)
file(GLOB_RECURSE CAMERACALIBRATOR_SRC "${CMAKE_CURRENT_SRC_DIR}" *.cpp)
add_executable(CameraCalibrator ${CAMERACALIBRATOR_HEADER} ${CAMERACALIBRATOR_SRC})

target_link_libraries(CameraCalibrator
	${OpenCV_LIBS}
	debug videoInput${CMAKE_DEBUG_POSTFIX} optimized videoInput
	debug realsense2${CMAKE_DEBUG_POSTFIX} optimized realsense2
)