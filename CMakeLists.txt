cmake_minimum_required(VERSION 2.8)
project(sls_roller)
include(SBELUtils.cmake)


SET(ENABLE_UNIT_GPU FALSE   CACHE   BOOL    "Turn on for GPU models")
SET(ENABLE_UNIT_MPI FALSE   CACHE   BOOL    "Turn on for GPU models")

# Additional packages
enable_openmp_support()

# User specified path to ChronoEngine
SET(CHRONO_INC       ""   CACHE PATH "Where is ChronoEngine Source located")

# Include directories based on user path to ChronoEngine
SET(INC_DIRS ${CHRONO_INC} "${CHRONO_INC}/collision" "${CHRONO_INC}/collision/bullet")

# User specified path to ChronoEngine Libraries
SET(CHRONO_LIB_PATH  ""   CACHE PATH "Where is CHRONOLIB located")

# Find required Libraries
FIND_LIBRARY(CHRONO_LIB NAMES ChronoEngine PATHS ${CHRONO_LIB_PATH} REQUIRED)

# Set unit-specific options
IF(ENABLE_UNIT_GPU)
    enable_cuda_support()
    LIST(APPEND INC_DIRS "${CUDA_TOOLKIT_ROOT_DIR}/include" "${CUDA_SDK_ROOT_DIR}/common/inc")

    FIND_LIBRARY(CHRONO_LIB_GPU NAMES ChronoEngine_GPU PATHS ${CHRONO_LIB_PATH} REQUIRED)
    FIND_LIBRARY(CHRONO_LIB_POSTPROCESS NAMES ChronoEngine_POSTPROCESS PATHS ${CHRONO_LIB_PATH} REQUIRED)
    FIND_LIBRARY(CHRONO_LIB_OPENGL NAMES ChronoEngine_OPENGL PATHS ${CHRONO_LIB_PATH} REQUIRED)

    SET(GPU_LIBS
        ${CHRONO_LIB_POSTPROCESS}
        ${CHRONO_LIB_GPU}
        ${CHRONO_LIB_OPENGL}
    )
ENDIF()

include_directories(${INC_DIRS})

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")
#SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ")

# Add executables
IF (ENABLE_UNIT_GPU)
    ADD_EXECUTABLE(sls sls.cpp)		
    TARGET_LINK_LIBRARIES(sls	${CHRONO_LIB} ${GPU_LIBS})

    ADD_EXECUTABLE(aor aor.cpp)		
    TARGET_LINK_LIBRARIES(aor	${CHRONO_LIB} ${GPU_LIBS})
   
ENDIF(ENABLE_UNIT_GPU)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

