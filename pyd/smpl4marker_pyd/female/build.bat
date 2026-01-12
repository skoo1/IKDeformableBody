@echo off

rem ================================
rem 1) CMake Configure
rem ================================
cmake -S . ^
      -B build ^
      -G "Visual Studio 17 2022" ^
      -A x64 ^
      -DCMAKE_BUILD_TYPE=Release ^
      -DCMAKE_PREFIX_PATH="." ^
      -DCMAKE_LIBRARY_OUTPUT_DIRECTORY="./build/bin" ^
      -DPYTHON_EXECUTABLE="C:/Users/HYkong/anaconda3/envs/SMPL/python.exe"


rem ================================
rem 2) CMake Build
rem ================================
cmake --build build --config Release