# pyd Code

The `pyd/` directory contains projects where **cost functions are implemented in C++** and built as `.pyd` files to accelerate **CMA-ES optimization**.

Each subdirectory corresponds to a specific module (e.g., `scan2smpl`, `smpl4marker`, `trc2skel`) and contains a `.pyd` build project for the **corresponding cost function**.

Details for each stage can be found in the corresponding `README.md` file located in each stage’s directory.

---

## Requirements

To build these projects, **CMake 3.5 or later** and **Microsoft Visual Studio 2022 (version 17 or later)** must be installed on the system.  
> ⚠️ **Note**  
> The Visual Studio version used for compilation can be adjusted in the `build.bat` file located in each `pyd` subdirectory, if necessary.

If needed, the C++ source code of a cost function (defined in `Environment.hpp`) can be modified, and the `.pyd` file can be rebuilt using the commands below.


The following shows how to build a `.pyd` file in **Release mode** using CMake on a Windows environment.  
*(`%CONDA_PREFIX%` refers to the currently active Anaconda environment.)*

---

## Repository Structure

This repository contains the following components:
```text
pyd/                     # C++-based cost functions (.pyd) for CMA-ES acceleration
├─ scan2smpl_pyd/
│  ├─ female/
│  │  ├─ thirdParty/
│  │  ├─ __init__.py
│  │  ├─ basicmodel_f_lbs_10_207_0_v1.1.0.json
│  │  ├─ build.bat
│  │  ├─ cfg.yaml
│  │  ├─ CMakeLists.txt
│  │  ├─ Common.hpp
│  │  ├─ debug_app.cpp
│  │  ├─ Environment.hpp
│  │  ├─ kdtree.cpp
│  │  ├─ kdtree.hpp
│  │  ├─ raisim_gym.cpp
│  │  ├─ RaisimGymEnv.hpp
│  │  ├─ RaisimGymVecEnv.py
│  │  ├─ runner.py
│  │  ├─ smpl.cpp
│  │  ├─ smpl.hpp
│  │  ├─ VectorizedEnvironment.hpp
│  │  ├─ Yaml.cpp
│  │  └─ Yaml.hpp
│  │
│  └─ male/
│     ├─ thirdParty/
│     ├─ __init__.py
│     ├─ basicmodel_m_lbs_10_207_0_v1.1.0.json
│     ├─ build.bat
│     ├─ cfg.yaml
│     ├─ CMakeLists.txt
│     ├─ Common.hpp
│     ├─ debug_app.cpp
│     ├─ Environment.hpp
│     ├─ kdtree.cpp
│     ├─ kdtree.hpp
│     ├─ raisim_gym.cpp
│     ├─ RaisimGymEnv.hpp
│     ├─ RaisimGymVecEnv.py
│     ├─ runner.py
│     ├─ smpl.cpp
│     ├─ smpl.hpp
│     ├─ VectorizedEnvironment.hpp
│     ├─ Yaml.cpp
│     └─ Yaml.hpp
│
├─ smpl4marker_pyd/
│  ├─ female/
│  │  ├─ thirdParty/
│  │  ├─ __init__.py
│  │  ├─ basicmodel_f_lbs_10_207_0_v1.1.0.json
│  │  ├─ build.bat
│  │  ├─ cfg.yaml
│  │  ├─ CMakeLists.txt
│  │  ├─ Common.hpp
│  │  ├─ debug_app.cpp
│  │  ├─ Environment.hpp
│  │  ├─ kdtree.cpp
│  │  ├─ kdtree.hpp
│  │  ├─ raisim_gym.cpp
│  │  ├─ RaisimGymEnv.hpp
│  │  ├─ RaisimGymVecEnv.py
│  │  ├─ runner.py
│  │  ├─ smpl.cpp
│  │  ├─ smpl.hpp
│  │  ├─ VectorizedEnvironment.hpp
│  │  ├─ Yaml.cpp
│  │  └─ Yaml.hpp
│  │
│  └─ male/
│     ├─ thirdParty/
│     ├─ __init__.py
│     ├─ basicmodel_m_lbs_10_207_0_v1.1.0.json
│     ├─ build.bat
│     ├─ cfg.yaml
│     ├─ CMakeLists.txt
│     ├─ Common.hpp
│     ├─ debug_app.cpp
│     ├─ Environment.hpp
│     ├─ kdtree.cpp
│     ├─ kdtree.hpp
│     ├─ raisim_gym.cpp
│     ├─ RaisimGymEnv.hpp
│     ├─ RaisimGymVecEnv.py
│     ├─ runner.py
│     ├─ smpl.cpp
│     ├─ smpl.hpp
│     ├─ VectorizedEnvironment.hpp
│     ├─ Yaml.cpp
│     └─ Yaml.hpp
│
├─ trc2skel_pyd/
│  ├─ female/
│  │  ├─ thirdParty/
│  │  ├─ __init__.py
│  │  ├─ build.bat
│  │  ├─ cfg.yaml
│  │  ├─ CMakeLists.txt
│  │  ├─ Common.hpp
│  │  ├─ debug_app.cpp
│  │  ├─ Environment.hpp
│  │  ├─ joints_def.cpp
│  │  ├─ joints_def.hpp
│  │  ├─ kdtree.cpp
│  │  ├─ kdtree.hpp
│  │  ├─ osim_joints.cpp
│  │  ├─ osim_joints.hpp
│  │  ├─ raisim_gym.cpp
│  │  ├─ RaisimGymEnv.hpp
│  │  ├─ RaisimGymVecEnv.py
│  │  ├─ runner.py
│  │  ├─ skel.cpp
│  │  ├─ skel.hpp
│  │  ├─ skel_female.json
│  │  ├─ VectorizedEnvironment.hpp
│  │  ├─ vnormal.cpp
│  │  ├─ vnormal.hpp
│  │  ├─ Yaml.cpp
│  │  └─ Yaml.hpp
│  │
│  └─ male/
│     ├─ thirdParty/
│     ├─ __init__.py
│     ├─ build.bat
│     ├─ cfg.yaml
│     ├─ CMakeLists.txt
│     ├─ Common.hpp
│     ├─ debug_app.cpp
│     ├─ Environment.hpp
│     ├─ joints_def.cpp
│     ├─ joints_def.hpp
│     ├─ kdtree.cpp
│     ├─ kdtree.hpp
│     ├─ osim_joints.cpp
│     ├─ osim_joints.hpp
│     ├─ raisim_gym.cpp
│     ├─ RaisimGymEnv.hpp
│     ├─ RaisimGymVecEnv.py
│     ├─ runner.py
│     ├─ skel.cpp
│     ├─ skel.hpp
│     ├─ skel_male.json
│     ├─ VectorizedEnvironment.hpp
│     ├─ vnormal.cpp
│     ├─ vnormal.hpp
│     ├─ Yaml.cpp
│     └─ Yaml.hpp
│
└─ README.md
```