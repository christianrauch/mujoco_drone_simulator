#!/usr/bin/env python3
import mujoco
import mujoco.viewer
from robot_descriptions.loaders.mujoco import load_robot_description


def main():
    model_name = "skydio_x2"
    m = load_robot_description(f"{model_name}_mj_description", variant="scene")
    d = mujoco.MjData(m)

    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            mujoco.mj_step(m, d)
            viewer.sync()

if __name__ == '__main__':
    main()
