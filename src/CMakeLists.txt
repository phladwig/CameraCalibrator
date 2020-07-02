#   CmakeLists for CameraCalibrator

include_directories(${CMAKE_BINARY_DIR}/src ${OpenCV_INCLUDE_DIRS})
link_directories(${CMAKE_BINARY_DIR}/lib "${CMAKE_BINARY_DIR}/lib")
file(GLOB_RECURSE CAMERACALIBRATOR_HEADER "${CMAKE_CURRENT_SRC_DIR}" *.h)
file(GLOB_RECURSE CAMERACALIBRATOR_SRC "${CMAKE_CURRENT_SRC_DIR}" *.cpp)
add_executable(CameraCalibrator ${CAMERACALIBRATOR_HEADER} ${CAMERACALIBRATOR_SRC})

target_link_libraries(CameraCalibrator
	${OpenCV_LIBS}	
)