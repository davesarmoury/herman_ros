from moveit_commander.conversions import list_to_pose
import copy

approach_height = 0.08
approach_offset = 0.05

positions = {}
positions["back"] = list_to_pose([0, 0, 0, 0, 0, 0])
positions["back_up"] = list_to_pose([0, 0, approach_height, 0, 0, 0])
positions["back_out"] = list_to_pose([-approach_offset, 0, 0, 0, 0, 0])
positions["back_out_up"] = list_to_pose([-approach_offset, 0, approach_height, 0, 0, 0])
positions["front"] = list_to_pose([0, 0, 0, 0, 0, 3.14159])
positions["front_up"] = list_to_pose([0, 0, approach_height, 0, 0, 3.14159])
positions["front_out"] = list_to_pose([approach_offset, 0, 0, 0, 0, 3.14159])
positions["front_out_up"] = list_to_pose([approach_offset, 0, approach_height, 0, 0, 3.14159])
positions["left"] = list_to_pose([0, 0, 0, 0, 0, 1.5707])
positions["left_up"] = list_to_pose([0, 0, approach_height, 0, 0, 1.5707])
positions["left_out"] = list_to_pose([0, -approach_offset, 0, 0, 0, 1.5707])
positions["left_out_up"] = list_to_pose([0, -approach_offset, approach_height, 0, 0, 1.5707])
positions["right"] = list_to_pose([0, 0, 0, 0, 0, -1.5707])
positions["right_up"] = list_to_pose([0, 0, approach_height, 0, 0, -1.5707])
positions["right_out"] = list_to_pose([0, approach_offset, 0, 0, 0, -1.5707])
positions["right_out_up"] = list_to_pose([0, approach_offset, approach_height, 0, 0, -1.5707])

positions["turn"] = list_to_pose([0, 0, 0, 0, 0, 0])
positions["turn_clockwise"] = list_to_pose([0, 0, 0, 1.5707, 0, 0])
positions["turn_counter"] = list_to_pose([0, 0, 0, -1.5707, 0, 0])
positions["turn_180"] = list_to_pose([0, 0, 0, 3.14159, 0, 0])
positions["turn_out"] = list_to_pose([-approach_offset, 0, 0, 0, 0, 0])
positions["turn_clockwise_out"] = list_to_pose([-approach_offset, 0, 0, 1.5707, 0, 0])
positions["turn_counter_out"] = list_to_pose([-approach_offset, 0, 0, -1.5707, 0, 0])
positions["turn_180_out"] = list_to_pose([-approach_offset, 0, 0, 3.14159, 0, 0])

home = [-1.347628694484477, -1.6605963965156643, -1.9764216397655545, -2.645967955802357, 0.0, 0.0]

def make_moves(arm, positions):
    for p in positions:
        waypoints = []
        waypoints.append(copy.deepcopy(p))

        count = 0
        while count < 20:
            (plan, fraction) = arm.compute_cartesian_path(waypoints, 0.01, 0.0)
            if fraction >= 0.99:
                print("Moving")
                arm.execute(plan)
                break
            else:
                print("Replanning")
            count = count + 1
        else:
            print("Replanning Failure")

def go_home(arm):
    arm.go(home)
    arm.stop()

def get_pick_moves(side):
    moves = []
    moves.append(positions[side + "_out_up"])
    moves.append(positions[side + "_out"])
    moves.append(positions[side])
    moves.append(positions[side + "_up"])
    return moves

def pick_rubik(arm, side):
    go_home(arm)
    arm.set_pose_reference_frame("fixture")
    moves = get_pick_moves(side)
    make_moves(arm, moves)

def place_rubik(arm, side):
    go_home(arm)
    arm.set_pose_reference_frame("fixture")
    moves = get_pick_moves(side)
    moves.reverse()
    make_moves(arm, moves)

def turn_rubik(arm, amount):
    go_home(arm)
    arm.set_pose_reference_frame("socket")
    arm.go(positions["turn_out"])
    moves = []
    moves.append(positions["turn"])
    moves.append(positions["turn_" + amount])
    moves.append(positions["turn_" + amount + "_out"])
    make_moves(arm, moves)
