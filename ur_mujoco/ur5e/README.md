# mujoco-model-of-ur5e-with-robotiq-2f-85

This repo has a mujoco model of ur5e with robotiq 2f 85 mounted on. These two models are adapted from [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) and combined here. Two sensors are added namely force/torque and gripper's finger tip.

![screenshot](https://github.com/abuibaid/mujoco-model-of-ur5e-with-robotiq-2f-85/assets/22854439/dd6daa99-d65c-4cff-9e39-a2f05f4ad06b)

## installation:
first install mujoco.viewer
```
pip install mujoco
```

then clone the repository to mujoco_ur5e folder

```
git clone https://github.com/abuibaid/mujoco-model-of-ur5e-with-robotiq-2f-85.git mujoco_ur5e
```

then call the folder using the following command
```
python3 -m mujoco.viewer --mjcf=~/mujoco_ur5e/ur5e.xml
```
