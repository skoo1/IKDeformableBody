# IKDeformableBody

This repository provides an **inverse kinematics (IK) pipeline for estimating human pose and global position** using **SMPL body models** and a **SKEL skeletal model**.  
Human scan data and motion capture data are used in the pipeline to perform SMPL–SKEL–based inverse kinematics.  
The body shape (β) of the SMPL model is estimated from human body scan data.  
Using the estimated shape, inverse kinematics of the SKEL model is performed based on motion capture data.

---

## What This Repository Provides

- **Estimation of SMPL shape parameters (β)** from human scan data
- **Preprocessing for estimating marker locations on the SMPL mesh** via SMPL pose and translation optimization using marker-attached scan data
- **SKEL-based inverse kinematics** using motion capture data
- Example datasets for testing and validation (`example/`)

---

## Repository Structure

This repository contains the following components:
```text
.
├─ scan2smpl/          # Estimate SMPL shape(β), pose, and translation from scan data
├─ smpl4marker/        # Align SMPL pose/translation using marker-attached scan data
├─ trc2skel/           # SKEL-based inverse kinematics using motion capture (.trc)
├─ pyd/                # C++-based cost functions (.pyd) for CMA-ES acceleration
│  ├─ scan2smpl/
│  ├─ smpl4marker/
│  └─ trc2skel/
├─ example/            # Example data for running and testing the code
└─ README.md
```
- Python 3.8 (Anaconda environment)
- Code for utilizing SMPL shape parameters (β) estimated from scan-to-SMPL fitting
- Code for computing SKEL-based inverse kinematics from motion capture data
- Example datasets provided in the `example/` directory

> **Note**  
> Due to licensing restrictions, SMPL and SKEL model files are **not included** in this repository and must be downloaded separately by the user.

---

## Model Preparation

To run this code, the **SMPL model and SKEL model must be downloaded in advance**.

- **SMPL model** : [download link](https://smpl.is.tue.mpg.de/download.php)  
  SMPL > Downloads > Download > Download version 1.0.0 for Python 2.7 (female/male. 10 shape PCs)    
- **SKEL model** : [download link](https://skel.is.tue.mpg.de/download.php)  
  SKEL > Downloads > SKEL and BSM models > Download Models  
- SMPL model python3 version : [GitHub link](https://github.com/DogeStudio/SMPL3)  
  Download this GitHub repository and place it inside the directory indicated in `SMPL3`.  

The downloaded models should be converted into a **JSON format** using the provided `XXXX.py` script before use.

---

## Code Structure and Functionality

The main pipeline consists of three stages.  
All optimization processes are based on **CMA-ES**, and `.pyd` files are used to accelerate computation.

- `scan2smpl/`  
  - Optimized parameters: **shape (β), pose, translation**
  - Fits the SMPL model to scan data to estimate body shape parameters

- `smpl4marker/`  
  - Optimized parameters: **pose, translation**
  - Estimates SMPL pose parameters to **map** marker locations on the SMPL surface

- `trc2skel/`  
  - Optimized parameters: **pose, translation**
  - Performs SKEL-based inverse kinematics using motion capture (`.trc`) data

The **shape parameter (β)** estimated in `scan2smpl` is reused and fixed in the subsequent `smpl4marker` and `trc2skel` stages.

---

## Notes

- This repository is provided for **research and experimental purposes**.
- The `example/` folder provides sample data and directory structure.
