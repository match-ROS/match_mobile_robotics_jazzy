"""Shared launch helpers for the integrated Cartesian admittance controller."""

from launch.substitutions import LaunchConfiguration


def launch_value(context, name):
    return LaunchConfiguration(name).perform(context)


def launch_bool(context, name):
    return launch_value(context, name).strip().lower() == "true"


def parse_float_list(value, expected_size, name):
    parts = str(value).replace(",", " ").split()
    if len(parts) != expected_size:
        raise RuntimeError(
            f"Launch argument '{name}' expects {expected_size} numeric values, "
            f"got {len(parts)}: {value}"
        )
    return [float(part) for part in parts]


def parse_semicolon_list(value):
    return [entry.strip() for entry in str(value).split(";") if entry.strip()]


def add_float_lists(*vectors):
    size = len(vectors[0])
    return [sum(vector[index] for vector in vectors) for index in range(size)]


def integrated_controller_inputs(context):
    use_ft_sensor = launch_bool(context, "integrated_controller_use_ft_sensor")
    reset_equilibrium = launch_value(
        context, "integrated_controller_reset_equilibrium_on_zero_command"
    ).strip().lower()
    if reset_equilibrium == "auto":
        reset_equilibrium_on_zero = not use_ft_sensor
    else:
        reset_equilibrium_on_zero = reset_equilibrium == "true"

    return {
        "use_ft_sensor": use_ft_sensor,
        "reset_equilibrium_on_zero": reset_equilibrium_on_zero,
        "admittance": parse_float_list(
            launch_value(context, "integrated_controller_admittance"),
            6,
            "integrated_controller_admittance",
        ),
        "wrench_sign": parse_float_list(
            launch_value(context, "integrated_controller_wrench_sign"),
            6,
            "integrated_controller_wrench_sign",
        ),
        "wrench_twist_gain": parse_float_list(
            launch_value(context, "integrated_controller_wrench_twist_gain"),
            6,
            "integrated_controller_wrench_twist_gain",
        ),
        "pose_error_gain": parse_float_list(
            launch_value(context, "integrated_controller_pose_error_gain"),
            6,
            "integrated_controller_pose_error_gain",
        ),
    }


def collision_forbidden_boxes(side, forbidden_box_arg):
    if not forbidden_box_arg:
        return []
    if forbidden_box_arg.lower() != "default":
        return parse_semicolon_list(forbidden_box_arg)
    opposite_side = "r" if side == "l" else "l"
    lift_columns = {
        "l": "UR10_l_lift_column:0.52,0.318,0.665:0.36,0.24,0.65",
        "r": "UR10_r_lift_column:0.52,-0.318,0.665:0.36,0.24,0.65",
    }
    return [
        "mir_chassis:0.0,0.0,0.25:1.00,0.68,0.50",
        "mir_top_surface_center:0.0,0.0,0.795:1.42,0.98,0.08",
        lift_columns[opposite_side],
    ]


def make_integrated_controller_params(
    context,
    *,
    robot_name,
    ur_l_xyz,
    ur_l_rpy,
    ur_r_xyz,
    ur_r_rpy,
):
    inputs = integrated_controller_inputs(context)
    ur_mount_xyz = {
        "l": parse_float_list(ur_l_xyz, 3, "ur_l_xyz"),
        "r": parse_float_list(ur_r_xyz, 3, "ur_r_xyz"),
    }
    ur_mount_rpy = {
        "l": parse_float_list(ur_l_rpy, 3, "ur_l_rpy"),
        "r": parse_float_list(ur_r_rpy, 3, "ur_r_rpy"),
    }
    ur_ideal_base_xyz = {
        "l": [0.549, 0.318, 0.832 - 0.49 + 0.555],
        "r": [0.549, -0.318, 0.832 - 0.49 + 0.555],
    }
    ur_collision_base_xyz = {
        side: add_float_lists(ur_ideal_base_xyz[side], ur_mount_xyz[side])
        for side in ("l", "r")
    }
    forbidden_box_arg = launch_value(
        context, "integrated_controller_collision_forbidden_boxes"
    ).strip()

    def params_for_side(side):
        arm_name = f"UR10_{side}"
        other_side = "r" if side == "l" else "l"
        other_arm_name = f"UR10_{other_side}"
        return {
            "robot_name": robot_name,
            "arm": side,
            "prefix": arm_name,
            "base_link": f"{arm_name}/base_link",
            "tip_link": f"{arm_name}/tool0",
            "debug_base_frame": f"{robot_name}/{arm_name}/base_link",
            "command_frame": f"{arm_name}/base_link",
            "equilibrium_frame": f"{robot_name}/{arm_name}/admittance_equilibrium_pose",
            "target_frame": f"{robot_name}/{arm_name}/admittance_target_pose",
            "joints": [
                f"{arm_name}/shoulder_pan_joint",
                f"{arm_name}/shoulder_lift_joint",
                f"{arm_name}/elbow_joint",
                f"{arm_name}/wrist_1_joint",
                f"{arm_name}/wrist_2_joint",
                f"{arm_name}/wrist_3_joint",
            ],
            "command_joints": [
                f"{arm_name}/shoulder_pan_joint",
                f"{arm_name}/shoulder_lift_joint",
                f"{arm_name}/elbow_joint",
                f"{arm_name}/wrist_1_joint",
                f"{arm_name}/wrist_2_joint",
                f"{arm_name}/wrist_3_joint",
            ],
            "use_ft_sensor": inputs["use_ft_sensor"],
            "require_wrench": launch_bool(context, "integrated_controller_require_wrench"),
            "wrench_in_tcp_frame": launch_bool(
                context, "integrated_controller_wrench_in_tcp_frame"
            ),
            "ft_sensor_name": f"{arm_name}/tcp_fts_sensor",
            "ft_state_interface_names": [
                f"{arm_name}/tcp_fts_sensor/force.x",
                f"{arm_name}/tcp_fts_sensor/force.y",
                f"{arm_name}/tcp_fts_sensor/force.z",
                f"{arm_name}/tcp_fts_sensor/torque.x",
                f"{arm_name}/tcp_fts_sensor/torque.y",
                f"{arm_name}/tcp_fts_sensor/torque.z",
            ],
            "command_timeout": float(launch_value(context, "integrated_controller_command_timeout")),
            "wrench_timeout": float(launch_value(context, "integrated_controller_wrench_timeout")),
            "wrench_bias_duration": float(
                launch_value(context, "integrated_controller_wrench_bias_duration")
            ),
            "wrench_filter_alpha": float(
                launch_value(context, "integrated_controller_wrench_filter_alpha")
            ),
            "force_deadband": float(launch_value(context, "integrated_controller_force_deadband")),
            "torque_deadband": float(launch_value(context, "integrated_controller_torque_deadband")),
            "admittance": inputs["admittance"],
            "wrench_twist_gain": inputs["wrench_twist_gain"],
            "pose_error_gain": inputs["pose_error_gain"],
            "wrench_sign": inputs["wrench_sign"],
            "max_linear_velocity": float(
                launch_value(context, "integrated_controller_max_linear_velocity")
            ),
            "max_angular_velocity": float(
                launch_value(context, "integrated_controller_max_angular_velocity")
            ),
            "max_joint_velocity": float(
                launch_value(context, "integrated_controller_max_joint_velocity")
            ),
            "max_joint_acceleration": float(
                launch_value(context, "integrated_controller_max_joint_acceleration")
            ),
            "max_joint_jerk": float(launch_value(context, "integrated_controller_max_joint_jerk")),
            "joint_limit_margin": float(
                launch_value(context, "integrated_controller_joint_limit_margin")
            ),
            "preserve_command_direction": launch_bool(
                context, "integrated_controller_preserve_command_direction"
            ),
            "immediate_zero_on_zero_command": launch_bool(
                context, "integrated_controller_immediate_zero_on_zero_command"
            ),
            "zero_command_deadband": float(
                launch_value(context, "integrated_controller_zero_command_deadband")
            ),
            "reset_equilibrium_on_zero_command": inputs["reset_equilibrium_on_zero"],
            "enable_collision_avoidance": launch_bool(
                context, "integrated_controller_enable_collision_avoidance"
            ),
            "collision_other_prefix": other_arm_name,
            "collision_other_base_link": f"{other_arm_name}/base_link",
            "collision_other_tip_link": f"{other_arm_name}/tool0",
            "collision_own_base_xyz": ur_collision_base_xyz[side],
            "collision_own_base_rpy": ur_mount_rpy[side],
            "collision_other_base_xyz": ur_collision_base_xyz[other_side],
            "collision_other_base_rpy": ur_mount_rpy[other_side],
            "collision_other_joint_names": [
                f"{other_arm_name}/shoulder_pan_joint",
                f"{other_arm_name}/shoulder_lift_joint",
                f"{other_arm_name}/elbow_joint",
                f"{other_arm_name}/wrist_1_joint",
                f"{other_arm_name}/wrist_2_joint",
                f"{other_arm_name}/wrist_3_joint",
            ],
            "collision_common_link": launch_value(
                context, "integrated_controller_collision_common_link"
            ),
            "collision_joint_states_topic": launch_value(
                context, "integrated_controller_collision_joint_states_topic"
            ),
            "collision_joint_state_timeout": float(
                launch_value(context, "integrated_controller_collision_joint_state_timeout")
            ),
            "collision_sample_spacing": float(
                launch_value(context, "integrated_controller_collision_sample_spacing")
            ),
            "collision_sphere_radius": float(
                launch_value(context, "integrated_controller_collision_sphere_radius")
            ),
            "collision_activation_clearance": float(
                launch_value(context, "integrated_controller_collision_activation_clearance")
            ),
            "collision_stop_clearance": float(
                launch_value(context, "integrated_controller_collision_stop_clearance")
            ),
            "collision_response_mode": launch_value(
                context, "integrated_controller_collision_response_mode"
            ),
            "collision_fail_safe_stop": launch_bool(
                context, "integrated_controller_collision_fail_safe_stop"
            ),
            "collision_forbidden_boxes": collision_forbidden_boxes(side, forbidden_box_arg),
            "publish_collision_markers": launch_bool(
                context, "integrated_controller_publish_collision_markers"
            ),
            "collision_marker_publish_rate_hz": float(
                launch_value(context, "integrated_controller_collision_marker_publish_rate_hz")
            ),
            "publish_state_rate_hz": 50.0,
        }

    return {
        "inputs": inputs,
        "ur_mount_rpy": ur_mount_rpy,
        "ur_collision_base_xyz": ur_collision_base_xyz,
        "params_by_side": {
            "l": params_for_side("l"),
            "r": params_for_side("r"),
        },
    }
