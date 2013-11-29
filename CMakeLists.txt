cmake_minimum_required(VERSION 2.8)
project(sls_roller)
include(cmake/ParallelUtils.cmake)

# Additional packages
enable_openmp_support()

# User specified path to ChronoEngine
SET(CHRONO_INC       ""   CACHE PATH "Where is ChronoEngine Source located")
SET(CHRONO_PARALLEL_INC       ""   CACHE PATH "Where is ChronoEngine Source located")
SET(CHRONO_OPENGL_INC  ""   CACHE PATH "Where is CHRONOOPENGL located")

# Include directories based on user path to ChronoEngine
SET(INC_DIRS ${CHRONO_INC} "${CHRONO_INC}/collision" "${CHRONO_INC}/collision/bullet" ${CHRONO_PARALLEL_INC} ${CHRONO_OPENGL_INC})

# User specified path to ChronoEngine Libraries
SET(CHRONO_LIB_PATH  ""   CACHE PATH "Where is CHRONOLIB located")
SET(CHRONO_PARALLEL_LIB_PATH  ""   CACHE PATH "Where is CHRONOPARALLEL located")
SET(CHRONO_OPENGL_LIB_PATH  ""   CACHE PATH "Where is CHRONOOPENGL located")
# Find required Libraries
FIND_LIBRARY(CHRONO_LIB NAMES ChronoEngine PATHS ${CHRONO_LIB_PATH} REQUIRED)

# Set unit-specific options

    enable_cuda_support()
    LIST(APPEND INC_DIRS "${CUDA_TOOLKIT_ROOT_DIR}/include" "${CUDA_SDK_ROOT_DIR}/common/inc")
    FIND_LIBRARY(CHRONO_LIB_PARALLEL NAMES ChronoEngine_Parallel PATHS ${CHRONO_PARALLEL_LIB_PATH} REQUIRED)
    #FIND_LIBRARY(CHRONO_LIB_POSTPROCESS NAMES ChronoEngine_POSTPROCESS PATHS ${CHRONO_LIB_PATH} REQUIRED)
    FIND_LIBRARY(CHRONO_LIB_OPENGL NAMES ChronoEngine_OPENGL PATHS ${CHRONO_OPENGL_LIB_PATH} REQUIRED)

    SET(GPU_LIBS
        #${CHRONO_LIB_POSTPROCESS}
        ${CHRONO_LIB_PARALLEL}
        ${CHRONO_LIB_OPENGL}
    )
find_package("GLUT")
find_package("OpenGL")
include_directories(${INC_DIRS} ${FREEGLUT_INC_PATH})


#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")
#SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ")

# Add executables

    ADD_EXECUTABLE(sls sls.cpp)		
    TARGET_LINK_LIBRARIES(sls	${CHRONO_LIB} ${GPU_LIBS})

    ADD_EXECUTABLE(aor aor.cpp)		
    TARGET_LINK_LIBRARIES(aor	${CHRONO_LIB} ${GPU_LIBS})
    
     ADD_EXECUTABLE(sls_post sls_post.cpp xyz_io.cpp )		
    TARGET_LINK_LIBRARIES(sls_post	${CHRONO_LIB} ${GPU_LIBS} ${FREEGLUT_LIB}
	${GLUT_LIBRARY}
	${OPENGL_LIBRARIES})
   
 ADD_EXECUTABLE(density density.cpp)		
    TARGET_LINK_LIBRARIES(density	${CHRONO_LIB} ${GPU_LIBS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

