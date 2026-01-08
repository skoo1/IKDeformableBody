# smpl4marker

This directory contains code for estimating the **SMPL pose and translation** from **marker-attached human body scan data** (`.obj`, `.ply`, `.stl`).

The SMPL model is aligned to the marker-attached scan data and used as a **preprocessing step to identify which vertices on the SMPL mesh correspond to the attached markers**.


---

## Shape Parameter (β)

To accurately reflect body shape, the **shape parameter (β) is not optimized** in this stage.
The shape parameter estimated in the `scan2smpl` stage is reused and treated as a fixed parameter in this stage.

---

## Optimized Parameters

The following parameters are optimized in this stage:

- pose
- translation (trans)

---

## Optimization Method

As in the `scan2smpl` stage, this module defines a cost function based on the

- **Euclidean distance**
- **Surface normal distance**

between the SMPL model surface and the scan data surface.

The pose and translation parameters are optimized using the CMA-ES algorithm.

---

## Cost Function (pyd)

The cost function used in this module is computed using a **C++ implementation compiled as a `.pyd` file**.

- pyd build location: `pyd/smpl4marker/`
- The `.pyd` file is used to accelerate CMA-ES–based optimization.

Instructions for building the `.pyd` file can be found in the README within the `pyd/smpl4marker/` directory.

---

## Requirements

- Python 3.8 (Anaconda environment)
- `SMPL3/` directory containing the Python implementation of the SMPL model
- Additional Python scripts and dependencies required to run this module

---

## Input and Output

### Input
- Marker-attached human body scan data (`.obj`)
  > Note: The scan data must be provided in meter units (vertex coordinates).
- Shape parameter (β) estimated in the `scan2smpl` stage (command input)

### Output
- Estimated SMPL parameters:
  - pose
  - translation (trans)
- Marker-to-vertex correspondence information on the SMPL model

---

## Output Usage

By applying the estimated

- pose
- translation (trans)
- shape (β) estimated in the `scan2smpl` stage

to the SMPL model, an SMPL mesh aligned with the marker-attached scan data can be obtained.

This alignment enables identification of the **corresponding SMPL vertices for each marker**.

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
  - `index_visual.py`
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
cd IKDeformableBody/smpl4marker
python smpl4marker.py --obj [obj_name] --gender [m/f] --betas ["b0 b1 b2 b3 b4 b5 b6 b7 b8 b9"]
```
> example
```bash
python smpl4marker.py --obj S011_Marker.obj --gender m --betas "1.17658723e+00  2.80551561e-01  1.09141893e+00  2.08250418e+00 -1.39071883e-01  1.43732296e-01 -7.20190481e-01  8.36525596e-02 3.26049776e-01  1.77389865e-03"
```

---

### Arguments
Arguments

- `--obj` : Name of the input scan OBJ file (with file extension or subject name)
- `--gender` : Gender of the SMPL model (-m : male, -f : female)
- `--betas` : betas obtained scan2smpl stage

---

### Optimization Result Visualization
Run the following command to evaluate the fitting error between the scan data and the fitted SMPL model and visualize the per-vertex distance:

```bash
python OBJdistance.py --obj [obj_name] --gender [m/f]
```
> example
```bash
python OBJdistance.py --obj S011_Scanner.obj --gender m
```

---

### SMPL Vertex Index (XYZ Coordinates)
Run the following command to compute and export SMPL vertex XYZ coordinates for a given subject.:

```bash
python index_visual.py --obj [obj_name] --gender [m/f]
```
> example
```bash
python index_visual.py --obj S011_Marker.obj --gender m
```
