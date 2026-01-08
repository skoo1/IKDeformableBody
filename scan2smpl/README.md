# scan2smpl

This directory contains code for estimating the **SMPL body shape (β), pose, and translation** from human body scan data (`.obj`, `.ply`, `.stl`).

The optimization minimizes a cost function defined by the **Euclidean distance** and **surface normal distance** between the SMPL model surface and the scan data surface.

---

## Optimized Parameters

The following parameters are optimized in this stage:

- shape (β)
- pose
- translation (trans)

---

## Optimization Method

This module defines a cost function based on the

- **Euclidean distance**
- **Surface normal distance**

between the SMPL model surface and the scan data surface.

The shape, pose and translation parameters are optimized using CMA-ES algorithm.

---

## Cost Function (pyd)

The cost function is computed using a **C++ implementation compiled as a `.pyd` file**.

- pyd build location: `pyd/scan2smpl/`
- The `.pyd` file is used to accelerate CMA-ES–based optimization.

Instructions for building the `.pyd` file can be found in the README within the `pyd/scan2smpl/` directory.

---

## Requirements

- Python 3.8 (Anaconda environment)
- `SMPL3/` directory containing the Python implementation of the SMPL model
- Additional Python scripts and dependencies required to run this module

---

## Input and Output

### Input
- Human body scan data (`.obj`, `.ply`, `.stl`)
- SMPL model files

### Output
- Estimated SMPL parameters:
  - shape (β)
  - pose
  - translation (trans)

<<<<<<< HEAD

How To Run
<여기 입력 파일, 출력 파일 경로 어떻게 할 지 코드 수정해서 추가 작성>
=======
---

## How To Run

Before running this code, make sure that the following files are located in the `scan2smpl/` directory.

### Required Files

- **SMPL model files**
  - `basicmodel_f_lbs_10_207_0_v1.1.0.json`
  - `basicmodel_f_lbs_10_207_0_v1.1.0.pkl`
  - `basicmodel_m_lbs_10_207_0_v1.1.0.json`
  - `basicmodel_m_lbs_10_207_0_v1.1.0.pkl`

- **Configuration / source files**
  - `cfg.yaml`
  - `OBJdistance.py`
  - `RaisimGymVecEnv.py`
  - `scan2smpl.py`

- **Cost function pyd files**
  - `scan2smpl_female.cp38-win_amd64.pyd`
  - `scan2smpl_male.cp38-win_amd64.pyd`

---

### Input / Output Structure

- **Scan data**  
  Input scan data (`.obj`) must be placed in the `scan_files/` directory.

- **Fitting results**  
  Fitting results are saved in the `results/` directory with the following files:
  - **3D object file (`.obj`)**  
    An OBJ file containing the optimized SMPL mesh
  - **SMPL parameter file (`.pkl`)**  
    A SMPL model file containing the optimized SMPL parameters
  - **Optimization log (`.txt`)**  
    A log file recording the optimization time and final parameter values

---

### Execution

Run the scan-to-SMPL fitting using the following command:

```bash
cd IKDeformableBody/scan2smpl
python scan2smpl.py --obj [obj_name] --gender [m/f]
```
---

### Arguments
Arguments

- `--obj` : Name of the input scan OBJ file (with file extension or subject name)
- `--gender` : Gender of the SMPL model (-m : male, -f : female)

---

### Optimization Result Visualization
Run the following command to evaluate the fitting error between the scan data and the fitted SMPL model and visualize the per-vertex distance:

```bash
python OBJdistance.py --obj [obj_name] --gender [m/f]
```
>>>>>>> d70e9ab (Update scan2smpl visualization and path handling)
