from moveit_commander.conversions import list_to_pose
import copy

approach_height = 0.08
approach_offset = 0.05

positions = {}
positions["back"] = list_to_pose([0, 0, 0, 0, 0, 0])
positions["back_up"] = list_to_pose([0, 0, approach_height, 0, 0, 0])
positions["back_out"] = list_to_pose([-approach_offset, 0, 0, 0, 0, 0])
positions["back_out_up"] = list_to_pose([-approach_offset, 0, approach_height, 0, 0, 0])
positions["left"] = list_to_pose([0, 0, 0, 0, 0, 1.5707])
positions["left_up"] = list_to_pose([0, 0, approach_height, 0, 0, 1.5707])
positions["left_out"] = list_to_pose([0, -approach_offset, 0, 0, 0, 1.5707])
positions["left_out_up"] = list_to_pose([0, -approach_offset, approach_height, 0, 0, 1.5707])
positions["right"] = list_to_pose([0, 0, 0, 0, 0, -1.5707])
positions["right_up"] = list_to_pose([0, 0, approach_height, 0, 0, -1.5707])
positions["right_out"] = list_to_pose([0, approach_offset, 0, 0, 0, -1.5707])
positions["right_out_up"] = list_to_pose([0, approach_offset, approach_height, 0, 0, -1.5707])

positions["back_turned"] = list_to_pose([0, 0, 0, 1.5707, 0, 0])
positions["back_turned_up"] = list_to_pose([0, 0, approach_height, 1.5707, 0, 0])
positions["back_turned_out"] = list_to_pose([-approach_offset, 0, 0, 1.5707, 0, 0])
positions["back_turned_out_up"] = list_to_pose([-approach_offset, 0, approach_height, 1.5707, 0, 0])

positions["turn"] = list_to_pose([0, 0, 0, 0, 0, 0])
positions["turn_clockwise"] = list_to_pose([0, 0, 0, 1.5707, 0, 0])
positions["turn_counter"] = list_to_pose([0, 0, 0, -1.5707, 0, 0])
positions["turn_out"] = list_to_pose([-approach_offset, 0, 0, 0, 0, 0])
positions["turn_clockwise_out"] = list_to_pose([-approach_offset, 0, 0, 1.5707, 0, 0])
positions["turn_counter_out"] = list_to_pose([-approach_offset, 0, 0, -1.5707, 0, 0])

home = [-1.347628694484477, -1.6605963965156643, -1.9764216397655545, -2.645967955802357, 0.0, 0.0]

camera_sides = [[-1.1330788771258753, -1.369629208241598, -2.0160892645465296, -2.8974650541888636, -2.7038405577289026, 0.0]
, [-0.9457677046405237, -1.9701207319842737, -1.3471739927874964, -2.965806309376852, 2.19595521452, -1.5707]
, [-0.9457677046405237, -1.9701207319842737, -1.3471739927874964, -2.965806309376852, 2.19595521452, 0]
, [-0.9457677046405237, -1.9701207319842737, -1.3471739927874964, -2.965806309376852, 2.19595521452, 1.5707]
, [-0.9457677046405237, -1.9701207319842737, -1.3471739927874964, -2.965806309376852, 2.19595521452, 3.14159]]

def make_moves(arm, robot, positions):
    for p in positions:
        waypoints = []
        waypoints.append(copy.deepcopy(p))

        count = 0
        while count < 40:
            (plan, fraction) = arm.compute_cartesian_path(waypoints, 0.001, 0.0)
            if fraction >= 0.99:
                plan = arm.retime_trajectory(robot.get_current_state(), plan, 1.0)
                if arm.execute(plan):
                    break
                else:
                    print("Execution Error")
            else:
                print("Planning Error")
            count = count + 1
        else:
            print("Failure")

def go_home(arm):
    arm.go(home)
    arm.stop()

def expose_up_down(arm, robot):
    pick_rubik(arm, robot, "back")
    place_rubik(arm, robot, "back_turned")

def reset_up_down(arm, robot):
    pick_rubik(arm, robot, "back_turned")
    place_rubik(arm, robot, "back")

def front_to_right(arm, robot):
    pick_rubik(arm, robot, "right")
    place_rubik(arm, robot, "back")

def right_to_front(arm, robot):
    pick_rubik(arm, robot, "back")
    place_rubik(arm, robot, "right")

def get_pick_moves(side):
    moves = []
    moves.append(positions[side + "_out_up"])
    moves.append(positions[side + "_out"])
    moves.append(positions[side])
    moves.append(positions[side + "_up"])
    return moves

def pick_rubik(arm, robot, side):
    arm.set_pose_reference_frame("fixture")
    arm.go(positions[side + "_out_up"])

    moves = []
    moves.append(positions[side + "_out"])
    moves.append(positions[side])
    moves.append(positions[side + "_up"])

    make_moves(arm, robot, moves)

def place_rubik(arm, robot, side):
    arm.set_pose_reference_frame("fixture")
    arm.go(positions[side + "_up"])

    moves = []
    moves.append(positions[side])
    moves.append(positions[side + "_out"])
    moves.append(positions[side + "_out_up"])

    make_moves(arm, robot, moves)

def turn_rubik(arm, robot, amount):
    arm.set_pose_reference_frame("socket")
    moves = []

    if "180" in amount:
        arm.go(positions["turn_clockwise_out"])

        moves.append(positions["turn_clockwise"])
        moves.append(positions["turn_counter"])
        moves.append(positions["turn_counter_out"])

    else:
        arm.go(positions["turn_out"])

        moves.append(positions["turn"])
        moves.append(positions["turn_" + amount])
        moves.append(positions["turn_" + amount + "_out"])

    make_moves(arm, robot, moves)
