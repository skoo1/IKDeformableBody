# pyd Code

## How To Run

Move to the target `.pyd` project directory:
```bash
cd D:\GitHub\ect\test\pyd\scan2smpl_pyd\female
cd D:\GitHub\ect\test\pyd\scan2smpl_pyd\male
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
del "IKDeformableBody\scan2smpl\scan2smpl_female.cp38-win_amd64.pyd"
copy "IKDeformableBody\pyd\scan2smpl_pyd\female\bin\scan2smpl_female.cp38-win_amd64.pyd" "IKDeformableBody\scan2smpl\scan2smpl_female.cp38-win_amd64.pyd"

del "IKDeformableBody\scan2smpl\scan2smpl_male.cp38-win_amd64.pyd"
copy "IKDeformableBody\pyd\scan2smpl_pyd\male\bin\scan2smpl_male.cp38-win_amd64.pyd" "IKDeformableBody\scan2smpl\scan2smpl_male.cp38-win_amd64.pyd"
```