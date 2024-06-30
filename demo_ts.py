import argparse

from omni.isaac.lab.app import AppLauncher

# create argparser
parser = argparse.ArgumentParser(description="Tutorial on spawning prims into the scene.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import omni.isaac.core.utils.prims as prim_utils

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg, RigidObject
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR
from omni.isaac.lab.sensors import CameraCfg, ContactSensorCfg, RayCasterCfg, patterns
from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg
from ts_cfg import DIFIT_CFG, SAW_DIGIT_R_CFG
from omni.isaac.lab_assets.anymal import ANYMAL_C_CFG  # isort: skip
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.math import quat_from_euler_xyz
from omni.isaac.lab.sim.spawners.lights import spawn_light, SphereLightCfg, DomeLightCfg
import torch
import numpy as np
from matplotlib import pyplot as plt
import cv2
from omni.isaac.lab.sim.spawners.shapes import SphereCfg
from omni.isaac.lab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.utils.math import subtract_frame_transforms
from img_utils import process_image_rgba

ori = [0, np.pi, -np.pi/2]
ori = quat_from_euler_xyz(torch.tensor(ori[0]),torch.tensor(ori[1]),torch.tensor(ori[2]))
ori = ori.detach().cpu().numpy()
ori = (ori[0], ori[1], ori[2], ori[3])

ori = [0, np.pi, -np.pi/2]
ori = quat_from_euler_xyz(torch.tensor(ori[0]),torch.tensor(ori[1]),torch.tensor(ori[2]))
ori = ori.detach().cpu().numpy()
ori = (ori[0], ori[1], ori[2], ori[3])


boder_ind = np.load("black_indices_expanded.npy")


X_RANGE = (0.45, 0.55)  # Example range, adjust as necessary
Y_RANGE = (-0.1, 0.1)  # Example range, adjust as necessary
Z_RANGE = (0.2, 0.15)  # Example range, adjust as necessary

@configclass
class DigitSceneCfg(InteractiveSceneCfg):
    """Designs the scene by spawning ground plane, light, objects and meshes from usd files."""
    # Ground-plane

    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    ## add digit
    digit: ArticulationCfg = SAW_DIGIT_R_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    box = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/Box", spawn=sim_utils.UsdFileCfg(usd_path="assets/box.usd",rigid_props=sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True),scale=(0.5,1,1)),init_state = RigidObjectCfg.InitialStateCfg(pos=(0.5, 0, 0)))

    cube = AssetBaseCfg(prim_path="{ENV_REGEX_NS}/Cube", spawn=sim_utils.UsdFileCfg(usd_path="assets/bolt.usda",rigid_props=sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=False),scale=(30,30,30)),init_state = RigidObjectCfg.InitialStateCfg(pos=(0.48, 0, 0.035)))
    # sphere = AssetBaseCfg(prim_path="{ENV_REGEX_NS}/Sphere", spawn=SphereCfg(radius=0.01,rigid_props=sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=False),mass_props = sim_utils.MassPropertiesCfg(),collision_props=sim_utils.CollisionPropertiesCfg()),init_state = RigidObjectCfg.InitialStateCfg(pos=(0.5, 0, 0.04)))

    camera_l = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/finger_left_tip_body/front_cam",
        update_period=0.01,
        height=320,
        width=240,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=115, focus_distance=0.0015, horizontal_aperture=20.955, clipping_range=(0.1, 1e5)
        ),
        offset=CameraCfg.OffsetCfg(pos=(0.003,0,-0.1),rot=ori, convention="opengl"),
    )
    camera_r = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/finger_right_tip_body/front_cam",
        update_period=0.01,
        height=320,
        width=240,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=115, focus_distance=0.0015, horizontal_aperture=20.955, clipping_range=(0.1, 1.0e5)
        ),
        offset=CameraCfg.OffsetCfg(pos=(0.003,0,-0.1),rot=ori, convention="opengl"),
    )

    camera_light_l = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Robot/finger_left_tip_body/L", spawn=sim_utils.UsdFileCfg(usd_path="assets/lights.usda"), init_state=AssetBaseCfg.InitialStateCfg(pos=(0.006, 0, -0.003)))
    
    camera_light_r = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Robot/finger_right_tip_body/L", spawn=sim_utils.UsdFileCfg(usd_path="assets/lights.usda"), init_state=AssetBaseCfg.InitialStateCfg(pos=(0.006, 0, -0.003)))
    
def process_depth(depth):
    depth[boder_ind[:, 0], boder_ind[:, 1]] = 0
    normalized_depth = cv2.normalize(depth, None, 0, 70000, cv2.NORM_MINMAX)
    normalized_depth = normalized_depth.astype(np.uint8)
    return normalized_depth

def process_rgb(rgb):
    rgb = rgb[:, :, :3]
    # rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    # cv2.imwrite("no.png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    rgb[boder_ind[:, 0], boder_ind[:, 1]] = [255, 255, 255]
    return rgb


def main():
    """Main function."""

    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, substeps=3)
    sim = sim_utils.SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([2.0, 0.0, 2.5], [-0.5, 0.0, 0.5])
    sim_dt = sim.get_physics_dt()

    # Design scene by adding assets to it
    scene_cfg = DigitSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    ENV_REGEX_NS = scene.env_regex_ns

    ## add a bunch of cubes or other objects in this way in interactive scene
    # for i in range(250):
    #     bolt = RigidObjectCfg(
    #         prim_path=f"{ENV_REGEX_NS}/Bolts_"+str(i+1),
    #         spawn=sim_utils.UsdFileCfg(
    #             usd_path="/home/xty/Downloads/base_link.usda",
    #             scale=(30, 30, 30),rigid_props=sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True,max_linear_velocity=0.05,max_angular_velocity=0.01),mass_props = sim_utils.MassPropertiesCfg(mass=0.02)
    #         ),
    #         init_state=RigidObjectCfg.InitialStateCfg(pos=(
    #             np.random.uniform(*X_RANGE),  # Random x
    #             np.random.uniform(*Y_RANGE),  # Random y
    #             np.random.uniform(*Z_RANGE)   # Random z
    #         ))
    #         )
    #     bolt = RigidObject(bolt)
    #     scene.rigid_objects[f"bolt_{i+1}"] = bolt

    # for i in range(150):
    #     sphere = RigidObjectCfg(prim_path=f"{ENV_REGEX_NS}/Bolts_"+str(i+1), spawn=SphereCfg(radius=0.01,rigid_props=sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True),mass_props = sim_utils.MassPropertiesCfg(),collision_props=sim_utils.CollisionPropertiesCfg()),init_state=RigidObjectCfg.InitialStateCfg(pos=(
    #             np.random.uniform(*X_RANGE),  # Random x
    #             np.random.uniform(*Y_RANGE),  # Random y
    #             np.random.uniform(*Z_RANGE)   # Random z
    #         )))
    #     sphere = RigidObject(sphere)

    #     scene.rigid_objects[f"bolt_{i+1}"] = sphere

    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Simulate physics
    count = 0
    robot = scene["digit"]

     # Create controller
    diff_ik_cfg = DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls")
    diff_ik_controller = DifferentialIKController(diff_ik_cfg, num_envs=scene.num_envs, device=sim.device)

    robot_entity_cfg = SceneEntityCfg("digit", joint_names=["right_j[0-6]"], body_names=["right_hand"])
    gripper_cfg = SceneEntityCfg("digit", joint_names=["base_joint_gripper_left","base_joint_gripper_right"], body_names=["right_hand"])
    gripper_cfg.resolve(scene)

    robot_entity_cfg.resolve(scene)
    ee_jacobi_idx = robot_entity_cfg.body_ids[0] - 1

    ee_goals = [
        [0.5, 0, 0.5, 0, 0, 1, 0]
    ]
    ee_goals = torch.tensor(ee_goals, device=sim.device)
    # Track the given command
    current_goal_idx = 0
    # Create buffers to store actions
    ik_commands = torch.zeros(scene.num_envs, diff_ik_controller.action_dim, device=robot.device)
    ik_commands[:] = ee_goals[current_goal_idx]

    while simulation_app.is_running():
        # reset
        # visualize the camera image
        img = scene["camera_r"].data.output["rgb"][0].detach().cpu().numpy()
        img = process_rgb(img)
        img = process_image_rgba(img, 'no_contact.png', 'bg_digit_240_320.jpg')
        img.save('output_image.jpg')

        if count % 450 == 0:
            # reset time
            count = 0
            # reset joint state
            joint_pos = robot.data.default_joint_pos.clone()
            joint_vel = robot.data.default_joint_vel.clone()
            # print(joint_pos[0])
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            robot.reset()
            # reset actions
            ik_commands[:] = ee_goals[current_goal_idx]
            joint_pos_des = joint_pos[:, robot_entity_cfg.joint_ids].clone()
            # reset controller
            diff_ik_controller.reset()
            diff_ik_controller.set_command(ik_commands)
            # change goal
            current_goal_idx = (current_goal_idx + 1) % len(ee_goals)
        else:
            # obtain quantities from simulation
            jacobian = robot.root_physx_view.get_jacobians()[:, ee_jacobi_idx, :, robot_entity_cfg.joint_ids]
            ee_pose_w = robot.data.body_state_w[:, robot_entity_cfg.body_ids[0], 0:7]
            root_pose_w = robot.data.root_state_w[:, 0:7]
            joint_pos = robot.data.joint_pos[:, robot_entity_cfg.joint_ids]
            # compute frame in root frame
            ee_pos_b, ee_quat_b = subtract_frame_transforms(
                root_pose_w[:, 0:3], root_pose_w[:, 3:7], ee_pose_w[:, 0:3], ee_pose_w[:, 3:7]
            )
            # compute the joint commands
            joint_pos_des = diff_ik_controller.compute(ee_pos_b, ee_quat_b, jacobian, joint_pos)
            # print(joint_pos_des)
            if count < 80:
                gripper_close = torch.tensor([-0.05,0.05], device=sim.device)
                gripper_close = gripper_close.unsqueeze(0).repeat(scene.num_envs, 1)
                robot.set_joint_position_target(gripper_close, joint_ids=gripper_cfg.joint_ids)
            elif count ==80:
                diff_ik_controller.reset()
                diff_ik_controller.set_command(torch.tensor([
                [0.5, 0, 0.17, 0, 0, 1, 0]
                    ]))
            elif 200 > count > 120:
                gripper_open = torch.tensor([-0.008,0.008], device=sim.device)
                gripper_open = gripper_open.unsqueeze(0).repeat(scene.num_envs, 1)
                robot.set_joint_position_target(gripper_open, joint_ids=gripper_cfg.joint_ids)
            elif 240 > count > 200:
                gripper_close = torch.tensor([-0.05,0.05], device=sim.device)
                gripper_close = gripper_close.unsqueeze(0).repeat(scene.num_envs, 1)
                robot.set_joint_position_target(gripper_close, joint_ids=gripper_cfg.joint_ids)
            elif 320 > count > 240:
                gripper_open = torch.tensor([-0.008,0.008], device=sim.device)
                gripper_open = gripper_open.unsqueeze(0).repeat(scene.num_envs, 1)
                robot.set_joint_position_target(gripper_open, joint_ids=gripper_cfg.joint_ids)
            elif 360> count > 320:
                gripper_close = torch.tensor([-0.05,0.05], device=sim.device)
                gripper_close = gripper_close.unsqueeze(0).repeat(scene.num_envs, 1)
                robot.set_joint_position_target(gripper_close, joint_ids=gripper_cfg.joint_ids)
            elif 400> count > 360:
                gripper_open = torch.tensor([-0.008,0.008], device=sim.device)
                gripper_open = gripper_open.unsqueeze(0).repeat(scene.num_envs, 1)
                robot.set_joint_position_target(gripper_open, joint_ids=gripper_cfg.joint_ids)
            elif count > 400:
                diff_ik_controller.reset()
                diff_ik_controller.set_command(torch.tensor([
                [0.5, 0, 0.3, 0, 0, 1, 0]
                    ]))
        robot.set_joint_position_target(joint_pos_des, joint_ids=robot_entity_cfg.joint_ids)

        scene.write_data_to_sim()
        sim.step()
        count += 1
        scene.update(0.01)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()