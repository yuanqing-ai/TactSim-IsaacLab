# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Franka Emika robots.

The following configurations are available:

* :obj:`FRANKA_PANDA_CFG`: Franka Emika Panda robot with Panda hand
* :obj:`FRANKA_PANDA_HIGH_PD_CFG`: Franka Emika Panda robot with Panda hand with stiffer PD control

Reference: https://github.com/frankaemika/franka_ros
"""

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR

##
# Configuration
##

SAW_DIGIT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="assets/sawyer_wsg50.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "right_j0": 0.0,
            "right_j1": -0.569,
            "right_j2": 0.0,
            "right_j3": -2.810,
            "right_j4": 0.0,
            "right_j5": -2.037,
            "right_j6": 0.741,
            "base_joint_gripper_right": 0.04,
            "base_joint_gripper_left": -0.04,
        },
    ),
    actuators={
        "panda_shoulder": ImplicitActuatorCfg(
            joint_names_expr=["right_j[0-6]","head_pan"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        # "panda_forearm": ImplicitActuatorCfg(
        #     joint_names_expr=["panda_joint[5-7]"],
        #     effort_limit=12.0,
        #     velocity_limit=2.61,
        #     stiffness=80.0,
        #     damping=4.0,
        # ),
        "panda_hand": ImplicitActuatorCfg(
            joint_names_expr=["base_joint_gripper_left","base_joint_gripper_right"],
            effort_limit=200.0,
            velocity_limit=0.2,
            stiffness=2e3,
            damping=1e2,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)


DIFIT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="assets/digit_asmb.usd",
        # usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/ANYbotics/ANYmal-D/anymal_d_minimal.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=4, solver_velocity_iteration_count=0,fix_root_link=True
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.02, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 1.1)),
    actuators={
        "digit_finger": ImplicitActuatorCfg(joint_names_expr=["digit_body_core_link"],
            effort_limit=0,
            velocity_limit=2,
            stiffness=2e3,
            damping=1e2,
        )
    },
)


SAW_DIGIT_R_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        # usd_path=f"/home/xty/Downloads/sawyer_wsg50 copy.usd",
        usd_path="assets/sawyer_wsg504.usda",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "right_j0": 0,
            "right_j1": -0.5,
            "right_j2": 0,
            "right_j3": 0,
            "right_j4": 1.5,
            "right_j5": -2.037,
            "right_j6": 1.57,
            "base_joint_gripper_right": 0.055,
            "base_joint_gripper_left": -0.055,
        },pos=(0.0, 0.0, 0)
    ),
    actuators={
        "panda_shoulder": ImplicitActuatorCfg(
            joint_names_expr=["right_j[0-6]"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=2e3,
            damping=1e2,
        ),
        # "panda_forearm": ImplicitActuatorCfg(
        #     joint_names_expr=["panda_joint[5-7]"],
        #     effort_limit=12.0,
        #     velocity_limit=2.61,
        #     stiffness=80.0,
        #     damping=4.0,
        # ),
        "panda_hand": ImplicitActuatorCfg(
            joint_names_expr=["base_joint_gripper_left","base_joint_gripper_right"],
            effort_limit=800.0,
            velocity_limit=0.00001,
            stiffness=1e6,
            damping=1e2,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
