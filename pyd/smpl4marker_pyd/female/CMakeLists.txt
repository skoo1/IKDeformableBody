cmake_minimum_required(VERSION 3.10)
project(_raisim_gym_torch)
set(CMAKE_CXX_STANDARD 17)

# 빌드 산출물 위치 지정
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

######################
### help functions ###
######################
macro(SUBDIRLIST result curdir)
    file(GLOB children RELATIVE ${curdir} ${curdir}/*)
    set(dirlist "")
    foreach(child ${children})
        if(IS_DIRECTORY ${curdir}/${child})
            list(APPEND dirlist ${child})
        endif()
    endforeach()
    set(${result} ${dirlist})
endmacro()

####################
### dependencies ###
####################
set(Dependencies)

# Pybind11
add_subdirectory(thirdParty/pybind11 pybind11)

# Eigen3
list(APPEND CMAKE_PREFIX_PATH ./thirdParty/Eigen3)
find_package(Eigen3 CONFIG REQUIRED)

# nlohmann_json
find_package(nlohmann_json 3.2.0 QUIET)
if (NOT nlohmann_json_FOUND)
    message(STATUS "nlohmann_json NOT FOUND. Fetching it via FetchContent...")
    include(FetchContent)
    FetchContent_Declare(
        json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG        v3.11.2
    )
    FetchContent_MakeAvailable(json)
endif()

# OpenMP 등 컴파일 옵션 분기
if(APPLE)
    # Mac의 경우, 기본 로직 그대로 유지
    set(CMAKE_C_FLAGS         "${CMAKE_C_FLAGS}         ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS       "${CMAKE_CXX_FLAGS}       ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

elseif(WIN32)
    # Windows (MSVC)에서는 /openmp /O2 /arch:AVX2 등을 사용
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /openmp /O2 /arch:AVX2")

else()
    # 리눅스 계열 (Unix-like)
    find_package(OpenMP REQUIRED)
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -fopenmp")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
endif()

# OS별로 raisim prefix path 추가
if(UNIX AND NOT APPLE AND NOT ANDROID AND NOT WEBGL AND NOT WIN32)
    set(RAISIM_OS linux)
    list(APPEND CMAKE_PREFIX_PATH ../raisim/${RAISIM_OS})

elseif(APPLE)
    # macOS/Apple 실사용 시 M1/Intel 분기
    execute_process(
        COMMAND uname -m
        RESULT_VARIABLE result
        OUTPUT_VARIABLE OSX_NATIVE_ARCHITECTURE
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    set(RAISIM_ARCH ${OSX_NATIVE_ARCHITECTURE} CACHE STRING "ARCH")

    if(${RAISIM_ARCH} STREQUAL "arm64")
        set(RAISIM_OS m1)
    else()
        set(RAISIM_OS mac)
    endif()

    list(APPEND CMAKE_PREFIX_PATH ../raisim/${RAISIM_OS})

elseif(WIN32)
    set(RAISIM_OS win32)
    list(APPEND CMAKE_PREFIX_PATH ../raisim/${RAISIM_OS})
    list(APPEND CMAKE_PREFIX_PATH ./thirdParty)
    list(APPEND Dependencies Ws2_32)

    # 릴리즈 산출물 경로 지정 (MSVC Multi-config)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE  ${CMAKE_CURRENT_SOURCE_DIR}/bin)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE  ${CMAKE_CURRENT_SOURCE_DIR}/bin)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE  ${CMAKE_CURRENT_SOURCE_DIR}/bin)
endif()


#######################
### src and linking ###
#######################

# Pybind 확장모듈 생성
pybind11_add_module(smpl4marker_female raisim_gym.cpp Yaml.cpp smpl.cpp kdtree.cpp)
list(APPEND Dependencies nlohmann_json::nlohmann_json)
target_link_libraries(smpl4marker_female PRIVATE ${Dependencies})
target_include_directories(smpl4marker_female PUBLIC ${EIGEN3_INCLUDE_DIRS})

# ====== [핵심] Windows / 비-Windows 분기 ======
if (MSVC)
    # MSVC용 최적화 및 LTO(원하면 /LTCG 추가) 등
    target_compile_options(smpl4marker_female PRIVATE /O2 /arch:AVX2)
else()
    # 리눅스/맥/GCC/Clang용
    target_compile_options(smpl4marker_female PRIVATE -fPIC -O3 -flto -mno-avx2)
endif()

# 각종 define
target_compile_definitions(smpl4marker_female PRIVATE "-DRAISIMGYM_TORCH_ENV_NAME=smpl4marker_female")
target_compile_definitions(smpl4marker_female PRIVATE "$<$<CONFIG:Release>:EIGEN_NO_DEBUG>")
target_compile_definitions(smpl4marker_female PRIVATE EIGEN_DONT_PARALLELIZE)

# 디버그용 메시지
message("raisim os ${RAISIM_OS}")
message("raisim arch ${RAISIM_ARCH}")
