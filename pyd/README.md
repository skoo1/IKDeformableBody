# pyd Code

The `pyd/` directory contains projects where **cost functions are implemented in C++** and built as `.pyd` files to accelerate **CMA-ES optimization**.

Each subdirectory corresponds to a specific module (e.g., `scan2smpl`, `smpl4marker`, `trc2skel`) and contains a `.pyd` build project for the **corresponding cost function**.

If needed, the C++ source code of a cost function can be modified, and the `.pyd` file can be rebuilt using the commands below.


The following shows how to build a `.pyd` file in **Release mode** using CMake on a Windows environment.
(`%CONDA_PREFIX%` refers to the currently active Anaconda environment.)

Details for each stage can be found in the corresponding `README.md` file located in each stage’s directory.