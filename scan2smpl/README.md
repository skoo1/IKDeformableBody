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


How To Run
<여기 입력 파일, 출력 파일 경로 어떻게 할 지 코드 수정해서 추가 작성>
