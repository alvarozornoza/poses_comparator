cmake_minimum_required(VERSION 2.6)
		
#project name
project(skViewer)


INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/core/)


#opencv is required to generate the solution
FIND_PACKAGE(OpenCV REQUIRED)

set(EXCECUTABLE_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})

#stop if using opencv < 2 (no cpp interface)
IF(${OpenCV_VERSION_MAJOR} LESS 2)
	message(SEND_ERROR "[ERROR] OpenCV major version is ${OpenCV_VERSION}. application was tested against opencv version > 2")
ENDIF(${OpenCV_VERSION_MAJOR} LESS 2)

include_directories(${OpenCV_INCLUDE_DIRS})

#source files list
file(
	GLOB_RECURSE
	source_files
	src/core/*.cpp
	src/core/*.c
	src/core/*.h	
	src/*.cpp
	src/*.h
    )

#generate executable from source files
add_executable(
	skViewer
	${source_files} 
)

#linking configurations
target_link_libraries(
	skViewer
	${OpenCV_LIBS}
	${OpenCV_LIBRARIES}
)

