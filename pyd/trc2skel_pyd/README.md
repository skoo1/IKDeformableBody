# pyd Code

## How To Run

Move to the target `.pyd` project directory:
```bash
cd IKDeformableBody\pyd\trc2skel_pyd\female
cd IKDeformableBody\pyd\trc2skel_pyd\male
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
cmake .. -DPYTHON_EXECUTABLE=%CONDA_PREFIX%\python.exe -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build . --config Release
```

---
## pyd code copy
> **Note**
> The `del` command is used **only when a previous** `.pyd` **file already exists** in the target directory.
> This ensures that the old binary is removed before copying the newly built `.pyd` file.

```bash
cd IKDeformableBody
del "trc2skel\trc2skel_female.cp38-win_amd64.pyd"
copy "pyd\trc2skel_pyd\female\bin\trc2skel_female.cp38-win_amd64.pyd" "trc2skel\trc2skel_female.cp38-win_amd64.pyd"

cd IKDeformableBody
del "trc2skel\trc2skel_male.cp38-win_amd64.pyd"
copy "pyd\trc2skel_pyd\male\bin\trc2skel_male.cp38-win_amd64.pyd" "trc2skel\trc2skel_male.cp38-win_amd64.pyd"
```

> If you are using Python 3.10, please follow the instructions in step 4,   
> “Rename the Generated .pyd File,” from the section below when executing the `copy` command:  
> [python 3.10 version](https://github.com/skoo1/IKDeformableBody/tree/main?tab=readme-ov-file#recommended-setup-python-310)