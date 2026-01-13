# trc2skel

This directory contains code for estimating the **SKEL model pose and translation** from motion capture data (`.trc`).

---

## Shape Parameter (β)

To accurately reflect body shape, the shape parameter is **not optimized** in this stage.  
The shape (β) estimated in the `scan2smpl` stage is reused and treated as a fixed parameter.

---

## Optimized Parameters

The following parameters are optimized in this stage:

- pose
- translation (trans)

---

## Optimization Method

This module defines a cost function based on the distance between:

- marker-corresponding vertices on the SMPL mesh (defined in `smpl4marker`), and
- motion capture marker positions.

The SKEL pose and translation parameters are optimized to minimize this cost using the CMA-ES algorithm.

---

## Cost Function (pyd)

The cost function used in this module is computed using a **C++ implementation compiled as a `.pyd` file**.

- pyd build location: `pyd/trc2skel/`
- The `.pyd` file is used to accelerate CMA-ES–based optimization.

Instructions for building the `.pyd` file can be found in the README within the `pyd/trc2skel/` directory.

---

## Requirements

- Python 3.8 (Anaconda environment)
- Additional Python scripts and dependencies required to run this module

---

## Input and Output

### Input
- Preprocessed motion capture data (`.trc`)
- Shape parameter (β) estimated in the `scan2smpl` stage
- Marker-to-vertex correspondence information (.yaml) defined in the `smpl4marker` stage

### Output
- Estimated SKEL parameters:
  - pose
  - translation (trans)

---

## Input Data Requirements

A **preprocessed motion capture `.trc` file** is required to run this code.

- The `.trc` file must be generated using the coordinate conventions of **Vicon Nexus** and **OpenSim**.
- Marker-to-vertex correspondence information must include all markers contained in the `.trc` file, and each marker must be mapped to a valid SMPL vertex index in the range **[0, 6889]**.
- The `.trc` file must contain position data for all markers **for every frame**. If any marker is missing in any frame, an error will occur.

---

## Output Usage

Using the estimated pose and translation parameters, inverse kinematics results for each joint of the SKEL model can be obtained.

---

## How To Run

Before running this code, make sure that the following files are located in the `trc2skel/` directory.

### Required Files

- **SKEL model files**
  - `skel_female.json`
  - `skel_male.json`

- **Configuration / source files**
  - `cfg.yaml`
  - `RaisimGymVecEnv.py`
  - `scan2smpl.py`

- **Cost function pyd files**
  - `trc2skel_female.cp38-win_amd64.pyd`
  - `trc2skel_male.cp38-win_amd64.pyd`

---

### Input / Output Structure

- **Motion capture data**  
  Input motion capture data (`.trc`) must be placed in the `trc/` directory.

- **Markerset data**  
  Input markerset data (`.yaml`) must be placed in the `markerset/` directory.

- **Fitting results**  
  Fitting results are saved in the `results/` directory with the following files:
  - **SKEL parameter file(`.npz`)**  
    A file that stores both intermediate (per-frame) optimization results and the final optimized SMPL parameters during the fitting process.
  - **SKEL parameter file (`.pkl`)**  
    A file that stores only the final optimized SMPL parameters after the optimization is completed.

---

### Execution

Run the trc-to-SKEL fitting using the following command:

```bash
cd IKDeformableBody/trc2skel
python trc2skel_pyd.py --subject [subject] --gender [m/f] --betas ["b0 b1 b2 b3 b4 b5 b6 b7 b8 b9"] --trc [trc_path] --markerset [markerset_path] --start_frame [start_frame_int] --end_frame [end_frame_int] 
```
> example
```bash
python trc2skel_pyd.py ^
  --subject S011 ^
  --gender male ^
  --betas "1.17658723e+00  2.80551561e-01  1.09141893e+00  2.08250418e+00 -1.39071883e-01  1.43732296e-01 -7.20190481e-01  8.36525596e-02 3.26049776e-01  1.77389865e-03" ^
  --trc trc/S011_walking01.trc ^
  --markerset markerset/S011_markerset.yaml ^
  --start-frame 1631 ^
  --end-frame 1787
```

---

### Arguments
Arguments

- `--obj` : Name of the input scan OBJ file (with file extension or subject name)
- `--gender` : Gender of the SMPL model (-m : male, -f : female)+
- `--betas` : Betas obtained scan2smpl stage
- `--trc` : Path of motion captrue data (`.trc`)
- `--markerset` : Path to the marker set corresponding to the SMPL model (`.yaml`)
- `--start-frame` : Number of start frame
- `--end-frame` : Number of end frame

---

### Convert SKEL to Kinematics (.csv)

Run the following command to convert the SKEL parameter file (`.pkl`) into a kinematics CSV file :


> **Note**  
> The SKEL `.pkl` file must be located inside the corresponding subject folder (e.g., `subjects/[subject_name]/`).

```bash
python skel2kinematics.py --subject [subject_name]
```
> example
```bash
python skel2kinematics.py --subject S011
```