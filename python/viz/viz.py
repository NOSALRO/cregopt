import argparse
import numpy as np
import RobotDART as rd
import dartpy

parser = argparse.ArgumentParser(description="Visualisation script for cregopt")
parser.add_argument(
    "--robot", type=str, help="Load robot model (biped, quad, hex)", default="quad"
)
parser.add_argument(
    "--terrain", type=str, help="Load terrain stl (flat, step)", default="flat"
)
parser.add_argument(
    "--data", type=str, help="Path to trajectory.txt data file", default=None
)
parser.add_argument(
    "--video", type=str, help="Save video to specified file", default=None
)
parser.add_argument("--replay", type=bool, help="Replay trajectory", default=True)
args = parser.parse_args()

simu = rd.RobotDARTSimu(0.05)

# graphics = rd.gui.Graphics()
gconfig = rd.gui.Graphics.default_configuration()
gconfig.width = 800
gconfig.height = 600
graphics = rd.gui.Graphics(gconfig)  # rd.gui.WindowlessGraphics(gconfig)

simu.set_graphics(graphics)
graphics.look_at([0.0, 8.0, 1.0], [1.5, 1.5, 0.5])

leg_dof_names_list = []
eefs_names = []
leg_mappings = []
robot = None
num_legs = 0
if args.robot == "biped":
    robot = rd.Robot(
        "robot_desc/xpp_hyq/biped.urdf",
        packages=[("xpp_hyq", "robot_desc/xpp_hyq")],
    )
    num_legs = 2

    leg_dof_names_list.append(["L_haa_joint", "L_hfe_joint", "L_kfe_joint"])
    leg_dof_names_list.append(["R_haa_joint", "R_hfe_joint", "R_kfe_joint"])

    eefs_names.append("L_tip")
    eefs_names.append("R_tip")

    leg_mappings.append(0)
    leg_mappings.append(1)
elif args.robot == "quad":
    robot = rd.Robot(
        "robot_desc/anymal_b_simple_description/anymal.urdf",
        packages=[
            (
                "anymal_b_simple_description",
                "robot_desc/anymal_b_simple_description",
            )
        ],
    )
    num_legs = 4

    leg_dof_names_list.append(["LF_HAA", "LF_HFE", "LF_KFE"])
    leg_dof_names_list.append(["LH_HAA", "LH_HFE", "LH_KFE"])
    leg_dof_names_list.append(["RF_HAA", "RF_HFE", "RF_KFE"])
    leg_dof_names_list.append(["RH_HAA", "RH_HFE", "RH_KFE"])

    eefs_names.append("LF_FOOT")
    eefs_names.append("LH_FOOT")
    eefs_names.append("RF_FOOT")
    eefs_names.append("RH_FOOT")

    leg_mappings.append(0)
    leg_mappings.append(2)
    leg_mappings.append(1)
    leg_mappings.append(3)
elif args.robot == "hex":
    robot = rd.Hexapod("robot_desc/pexod/my_pexod.urdf")
    num_legs = 6

    leg_dof_names_list.append(["body_leg_0", "leg_0_1_2", "leg_0_2_3"])
    leg_dof_names_list.append(["body_leg_1", "leg_1_1_2", "leg_1_2_3"])
    leg_dof_names_list.append(["body_leg_2", "leg_2_1_2", "leg_2_2_3"])
    leg_dof_names_list.append(["body_leg_3", "leg_3_1_2", "leg_3_2_3"])
    leg_dof_names_list.append(["body_leg_4", "leg_4_1_2", "leg_4_2_3"])
    leg_dof_names_list.append(["body_leg_5", "leg_5_1_2", "leg_5_2_3"])

    eefs_names.append("leg_0_tip")
    eefs_names.append("leg_1_tip")
    eefs_names.append("leg_2_tip")
    eefs_names.append("leg_3_tip")
    eefs_names.append("leg_4_tip")
    eefs_names.append("leg_5_tip")

    leg_mappings.append(4)
    leg_mappings.append(2)
    leg_mappings.append(0)
    leg_mappings.append(1)
    leg_mappings.append(3)
    leg_mappings.append(5)

simu.add_visual_robot(robot)
robot.set_cast_shadows(True)

if args.terrain == "step":
    floor = rd.Robot("terrain/step_floor.urdf", "step_floor")
    simu.add_visual_robot(floor)
    floor.set_base_pose([0.0, 0.0, 0.0, -0.17, 0.0, 0.0])
    floor.set_cast_shadows(True)
else:
    simu.add_checkerboard_floor(20.0)

data = np.loadtxt(args.data)

dof_names = (
    robot.dof_names()
)  # ['rootJoint_rot_x', 'rootJoint_rot_y', 'rootJoint_rot_z', 'rootJoint_pos_x', 'rootJoint_pos_y', 'rootJoint_pos_z', 'LF_HAA', 'LF_HFE', 'LF_KFE', 'LH_HAA', 'LH_HFE', 'LH_KFE', 'RF_HAA', 'RF_HFE', 'RF_KFE', 'RH_HAA', 'RH_HFE', 'RH_KFE']

robot.set_color_mode("material")


def log_rotation(R):
    theta = np.arccos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))
    if theta < 1e-12:
        return np.zeros((3, 1))
    mat = R - R.T
    r = np.array([mat[2, 1], mat[0, 2], mat[1, 0]]).reshape((3, 1))
    return theta / (2.0 * np.sin(theta)) * r


# function for damped pseudo-inverse
def pinv(jac, l=0.01):
    m, n = jac.shape
    if n >= m:
        return jac.T @ np.linalg.inv(jac @ jac.T + l * l * np.eye(m))
    return np.linalg.inv(jac.T @ jac + l * l * np.eye(n)) @ jac.T


def angle_dist(a, b):
    theta = b - a
    while theta < -np.pi:
        theta += 2 * np.pi
    while theta > np.pi:
        theta -= 2 * np.pi
    return theta


def angle_wrap_multi(theta):
    th = np.copy(theta)
    for i in range(theta.shape[0]):
        th[i] = angle_dist(0, th[i])
    return th


def ik_jac(
    robot,
    init_positions,
    pos_desired,
    eef_link_name,
    leg_dof_names,
    step=np.pi / 2.0,
    max_iter=200,
    min_error=1e-6,
):
    positions = np.copy(init_positions)
    f_error = 0.0
    pd = np.copy(pos_desired)
    success = False
    # pd[2] -= 0.025
    for i in range(max_iter):
        robot.set_positions(positions, leg_dof_names)
        tf = robot.body_pose(eef_link_name)
        error_in_world_frame = np.zeros(6)
        error_in_world_frame[3:] = pd - tf.translation()

        f_error = np.linalg.norm(error_in_world_frame)
        if f_error < min_error:
            success = True
            break

        J = robot.jacobian(eef_link_name, leg_dof_names)
        J_pinv = pinv(J)

        dq = J_pinv @ error_in_world_frame

        for j in range(dq.shape[0]):
            if dq[j] > step:
                dq[j] = step
            elif dq[j] < -step:
                dq[j] = -step
        positions += dq
    positions = angle_wrap_multi(positions)

    return positions, success


leg_init_positions = []

first = True

if args.video is not None:
    graphics.record_video("args.video")

index = 0
while True:
    r = data[index, :]
    if simu.step_world(False, False):
        break
    if simu.schedule(20):
        body_pos = np.copy(r[:3])
        body_rot = np.copy(r[3:6])

        # body_pos[2] += 0.025/2.
        robot.set_positions(body_pos, dof_names[3:6])

        R = dartpy.math.eulerZYXToMatrix(body_rot[::-1])
        robot.set_positions(log_rotation(R), dof_names[:3])

        for i in range(num_legs):
            k = leg_mappings[i]
            foot_pos = r[6 + k * 3 : 6 + (k + 1) * 3]
            if first:
                pos, _ = ik_jac(
                    robot,
                    np.copy(robot.positions(leg_dof_names_list[i])),
                    foot_pos,
                    eefs_names[i],
                    leg_dof_names_list[i],
                )
                robot.set_positions(pos, leg_dof_names_list[i])
                leg_init_positions.append(
                    np.copy(robot.positions(leg_dof_names_list[i]))
                )
            else:
                pos = np.copy(robot.positions(leg_dof_names_list[i]))
                p, s = ik_jac(
                    robot,
                    leg_init_positions[i],
                    foot_pos,
                    eefs_names[i],
                    leg_dof_names_list[i],
                )
                if s:
                    pos = np.copy(p)
                robot.set_positions(pos, leg_dof_names_list[i])
        first = False
        index += 1
        if index >= data.shape[0]:
            if args.replay:
                index = 0
            else:
                index = data.shape[0] - 1
