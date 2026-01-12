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
- Example datasets for testing and validation (`examples/`)

---

## Repository Structure

This repository contains the following components:
```text
.
├─ pkl2json/                # Convert .pkl models to JSON format
│  ├─ pkl_files/            
│  ├─ json_files/           
│  └─ pkl2json.py           
│
├─ pyd/                     # C++-based cost functions (.pyd) for CMA-ES acceleration
│  ├─ scan2smpl_pyd/
│  │  ├─ female/
│  │  └─ male/
│  │
│  ├─ smpl4marker_pyd/
│  │  ├─ female/
│  │  └─ male/
│  │
│  └─ trc2skel_pyd/
│      ├─ female/
│      └─ male/
│
├─ scan2smpl/               # Estimate SMPL shape(β), pose, and translation from scan data
│  ├─ SMPL3/
│  ├─ scan_files/
│  ├─ results/
│  ├─ basicmodel_f_lbs_10_207_0_v1.1.0.json
│  ├─ basicmodel_f_lbs_10_207_0_v1.1.0.pkl
│  ├─ basicmodel_m_lbs_10_207_0_v1.1.0.json
│  ├─ basicmodel_m_lbs_10_207_0_v1.1.0.pkl
│  ├─ cfg.yaml
│  ├─ scan2smpl_female.cp38-win_amd64.pyd
│  ├─ scan2smpl_male.cp38-win_amd64.pyd
│  ├─ RaisimGymVecEnv.py
│  ├─ scan2smpl.py
│  └─ OBJdistance.py
│
├─ smpl4marker/        # Align SMPL pose/translation using marker-attached scan data
│  ├─ SMPL3/
│  ├─ scan_files/
│  ├─ results/
│  ├─ basicmodel_f_lbs_10_207_0_v1.1.0.json
│  ├─ basicmodel_f_lbs_10_207_0_v1.1.0.pkl
│  ├─ basicmodel_m_lbs_10_207_0_v1.1.0.json
│  ├─ basicmodel_m_lbs_10_207_0_v1.1.0.pkl
│  ├─ cfg.yaml
│  ├─ scan2smpl_female.cp38-win_amd64.pyd
│  ├─ scan2smpl_male.cp38-win_amd64.pyd
│  ├─ RaisimGymVecEnv.py
│  ├─ scan2smpl.py
│  ├─ OBJdistance.py
│  └─ index_visual.py
│
├─ trc2skel/           # SKEL-based inverse kinematics using motion capture (.trc)
│  ├─ markerset/
│  ├─ trc/
│  ├─ results/
│  ├─ cfg.yaml
│  ├─ RaisimGymVecEnv.py
│  ├─ skel_female.json
│  ├─ skel_male.json
│  ├─ trc2skel_female.cp38-win_amd64.pyd
│  ├─ trc2skel_male.cp38-win_amd64.pyd
│  └─ trc2skel.py
│
├─ examples/            # Example data for running and testing the code
│  ├─ S011_Marker.obj            # Using smpl4marker stage
│  ├─ S011_markerset.yaml        # Using trc2skel stage
│  ├─ S011_Scanner.obj           # Using scan2smpl stage
│  ├─ S011_walking01.trc         # Using trc2skel stage
│  ├─ S015_Marker.obj            # Using smpl4marker stage
│  ├─ S015_markerset.yaml        # Using trc2skel stage
│  ├─ S015_Scanner.obj           # Using scan2smpl stage
│  └─ S015_walking01.trc         # Using trc2skel stage
│
└─ README.md
```
- Python 3.8 (Anaconda environment)
- Code for utilizing SMPL shape parameters (β) estimated from scan-to-SMPL fitting
- Code for computing SKEL-based inverse kinematics from motion capture data
- Example datasets provided in the `examples/` directory

> **Note**  
> Due to licensing restrictions, SMPL and SKEL model files are **not included** in this repository and must be downloaded separately by the user.

---

## Model Preparation

To run this code, the **SMPL and SKEL models must be downloaded in advance**.

  > **Important**: This project requires the correct model. If you use the wrong model, it may not run properly or may produce incorrect results.
- **SMPL model** : [download link](https://smpl.is.tue.mpg.de/download.php)  
  SMPL > Downloads > Download > Download version 1.1.0 for Python 2.7 (female/male/neutral, 300 shape PCs)
- **SKEL model** : [download link](https://skel.is.tue.mpg.de/download.php)  
  SKEL > Downloads > SKEL and BSM models > Download Models  
- **SMPL model python3 version** : [GitHub link](https://github.com/DogeStudio/SMPL3)  
  Download this GitHub repository and place it inside the directory indicated in `SMPL3`.  
  > The `SMPL3` directory location is specified in the *Repository Structure* section above.

After downloading, make sure that the following **four `.pkl` files** are placed in `pkl2json/pkl_files/`:
| Model | File name |
|------|-----------|
| SMPL male   | `basicmodel_m_lbs_10_207_0_v1.1.0.pkl` |
| SMPL female | `basicmodel_f_lbs_10_207_0_v1.1.0.pkl` |
| SKEL male   | `skel_male.pkl` |
| SKEL female | `skel_female.pkl` |

Then convert them to `.json` using the following command. The converted `.json` files will be saved in `pkl2json/json_files/`.

```bash
python pkl2json/pkl2json.py
```
> For each stage, the required `.pkl` files and the converted `.json` files are stored in the corresponding folder.

---

## Code Structure and Functionality

The main pipeline consists of three stages.  
All optimization processes are based on **CMA-ES**, and `.pyd` files are used to accelerate computation.

- `scan2smpl/`  
  - Optimized parameters: **shape(*β*), pose(*θ*), translation(*t*)**
  - Fits the SMPL model to scan data to estimate body shape parameters

- `smpl4marker/`  
  - Optimized parameters: **pose(*θ*), translation(*t*)**
  - Estimates SMPL pose parameters to **map** marker locations on the SMPL surface

- `trc2skel/`  
  - Optimized parameters: **pose(*θ*), translation(*t*)**
  - Performs SKEL-based inverse kinematics using motion capture (`.trc`) data

The **shape parameter(*β*)** estimated in `scan2smpl` is reused and fixed in the subsequent `smpl4marker` and `trc2skel` stages.

---

## How to Run
Because Python package versions have a significant impact on execution, this project should be run **inside an Anaconda environment**.
You can set up the environment using the following commands:
```bash
conda create -n IKDeformableBody python=3.8
conda activate IKDeformableBody
pip install -r requirements.txt
```
> **Notes** : If you change the Python version, you must also rebuild the .pyd modules with the corresponding Python version.

After setting up the environment, run the pipeline in the following order:
1. `scan2smpl`
2. `smpl4marker`
3. `trc2skel`
   (For detailed instructions, please refer to the `README.md` file in each corresponding folder.)

---

## Notes

- This repository is provided for **research and experimental purposes**.
- The `examples/` folder provides sample data and directory structure.
