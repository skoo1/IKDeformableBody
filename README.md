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
│     ├─ female/
│     └─ male/
│
├─ scan2smpl/               # Estimate SMPL shape(β), pose, and translation from scan data
│  ├─ README.md
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
│  ├─ README.md
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
│  ├─ README.md
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

Before proceeding with model preparation, make sure the repository has been cloned (*`git clone`*) and the environment has been set up as described in **How to Run**.

To run this code, the **SMPL and SKEL models must be downloaded in advance**.

> **Important**: This project requires the correct model. If you use the wrong model, it may not run properly or may produce incorrect results.
- **SMPL model** : [download link](https://smpl.is.tue.mpg.de/download.php)  
  SMPL > Downloads > Download > Download version 1.1.0 for Python 2.7 (female/male/neutral, 300 shape PCs)
- **SKEL model** : [download link](https://skel.is.tue.mpg.de/download.php)  
  SKEL > Downloads > SKEL and BSM models > Download Models  
- **SMPL model python3 version** : [GitHub link](https://github.com/DogeStudio/SMPL3)  
  Download this GitHub repository and place it inside the directory indicated in `SMPL3`.  
  > The `SMPL3` directory location is specified in the *Repository Structure* section above.  
  > (i.e., `scan2smpl/SMPL3` and `smpl4marker/SMPL3`).

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
> **Tested Environment**  
> This repository has been tested on **Windows 10** and **Windows 11** (verified on January 14, 2026).

Because Python package versions have a significant impact on execution, this project should be run **inside an Anaconda environment**.

First, clone the repository:
```bash
git clone https://github.com/skoo1/IKDeformableBody.git  
cd IKDeformableBody 
```
> **Note**  
> When using an outdated version of Git, a warning may be issued during the cloning process, and the checkout step may not complete as expected.  
> If this occurs, please update Git to version 2.45.2 or later, and then re-run the `git clone` command.

Then, create and activate the Anaconda environment:
```bash
conda create -n IKDeformableBody python=3.8
conda activate IKDeformableBody
```

Finally, install the required Python packages:
```bash
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

---

## Python Version Compatibility

### Supported Python Versions
This repository was originally developed and tested with **Python 3.8**.  
It can be used with **Python versions up to 3.10**, but there are important constraints.
 - ✅ **Python 3.8 – 3.10**: Supported
 - ❌ **Python 3.11 and later**: Not supported
When using **Python 3.10**, additional manual steps are required (see below).


### Notes on `chumpy`
- For **Python 3.10**, `chumpy` is **not installed automatically**.
- You must install `chumpy` **manually from the GitHub source**.
- Starting from **Python 3.11**, building the cost function `.pyd` files fails due to incompatibilities between Python’s C API and the bundled C++/pybind11 code.

⚠️ **Python 3.11+ is not supported** because the `.pyd` (C++ extension) build process produces compilation errors.

<br>

### 📌Recommended Setup (Python 3.10)
**1. Clone the Repository**
```bash
git clone https://github.com/skoo1/IKDeformableBody.git
cd IKDeformableBody
```

**2. Create and Activate a Conda Environment**
```bash
conda create -n IKDeformableBody python=3.10
conda activate IKDeformableBody
```

**3. Install `chumpy` Manually**
Before installing other dependencies, **remove `chumpy` from `requirements.txt`**.  
Then install `chumpy` directly from its GitHub repository:
```bash
python -m pip install --no-build-isolation "chumpy @ git+https://github.com/mattloper/chumpy.git"
```

After that, install the remaining dependencies:
```bash
pip install -r requirements.txt
```

**4. Rename the Generated `.pyd` File**
After building, you must rename and copy the generated `.pyd` file so that it matches
your Python version.

General format:
```bash
copy "pyd\scan2smpl_pyd\male\bin\scan2smpl_male.cp[PYTHON_VERSION]-win_amd64.pyd" "scan2smpl\scan2smpl_male.cp[PYTHON_VERSION]-win_amd64.pyd"
copy "pyd\smpl4marker_pyd\male\bin\smpl4marker_male.cp[PYTHON_VERSION]-win_amd64.pyd" "smpl4marker\smpl4marker_male.cp[PYTHON_VERSION]-win_amd64.pyd"
copy "pyd\trc2skel_pyd\male\bin\trc2skel_male.cp[PYTHON_VERSION]-win_amd64.pyd" "trc2skel\trc2skel_male.cp[PYTHON_VERSION]-win_amd64.pyd"
```

> **Example (Python 3.10)**
```bash
copy "pyd\scan2smpl_pyd\male\bin\scan2smpl_male.cp310-win_amd64.pyd" "scan2smpl\scan2smpl_male.cp310-win_amd64.pyd"
copy "pyd\smpl4marker_pyd\male\bin\smpl4marker_male.cp310-win_amd64.pyd" "smpl4marker\smpl4marker_male.cp310-win_amd64.pyd"
copy "pyd\trc2skel_pyd\male\bin\trc2skel_male.cp310-win_amd64.pyd" "trc2skel\trc2skel_male.cp310-win_amd64.pyd"
```
Make sure the `cpXXX` tag matches your Python version exactly.


### Summary
- This project must use **Python ≤ 3.10**
- `chumpy` must be **installed manually** for Python 3.10
- Python 3.11+ is **not supported** due to C++/pybind11 build errors
- The generated `.pyd` file must be renamed to match the Python version