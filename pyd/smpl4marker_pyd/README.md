# pyd Code

## How To Run

Move to the target `.pyd` project directory:
```bash
cd IKDeformableBody\pyd\smpl4marker_pyd\female
cd IKDeformableBody\pyd\smpl4marker_pyd\male
```

### Clean and build
> **Note**
> The `rmdir` commands below are used only to remove existing build artifacts (bin/, build/) from previous builds.
> If these directories do not exist, the commands can be skipped.
```bash
rmdir /s /q bin
rmdir /s /q build
mkdir build
cd build
cmake .. -DPYTHON_EXECUTABLE=%CONDA_PREFIX%\python.exe
cmake --build . --config Release
```

---
## pyd code copy
> **Note**
> The `del` command is used **only when a previous** `.pyd` **file already exists** in the target directory.
> This ensures that the old binary is removed before copying the newly built `.pyd` file.

```bash
cd IKDeformableBody
del "smpl4marker\smpl4marker_female.cp38-win_amd64.pyd"
copy "pyd\smpl4marker_pyd\female\bin\smpl4marker_female.cp38-win_amd64.pyd" "smpl4marker\smpl4marker_female.cp38-win_amd64.pyd"

cd IKDeformableBody
del "smpl4marker_pyd\smpl4marker_male.cp38-win_amd64.pyd"
copy "pyd\smpl4marker_pyd\male\bin\smpl4marker_male.cp38-win_amd64.pyd" "smpl4marker\smpl4marker_male.cp38-win_amd64.pyd"
```