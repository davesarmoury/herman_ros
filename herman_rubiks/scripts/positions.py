from moveit_commander.conversions import list_to_pose
import copy

approach_height = 0.05
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

home = [-1.347628694484477, -1.6605963965156643, -1.9764216397655545, -2.645967955802357, 0.0, 0.0]

def make_moves(arm, positions):
    for p in positions:
        waypoints = []
        waypoints.append(copy.deepcopy(p))

        while True:
            (plan, fraction) = arm.compute_cartesian_path(waypoints, 0.01, 0.0)
            if fraction >= 0.99:
                print("Moving")
                arm.execute(plan)
                break
            else:
                print("Replanning")

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
    moves = get_pick_moves(side)
    make_moves(arm, moves)

def place_rubik(arm, side):
    go_home(arm)
    moves = get_pick_moves(side).reverse()
    make_moves(arm, moves)

def turn_clockwise(arm):
    go_home(arm)
    moves = []
    moves.append(positions["turn_pounce"])
    moves.append(positions["turn_insert"])
    moves.append(positions["turn_clockwise"])
    moves.append(positions["turn_pounce_clockwise"])
    make_moves(arm, moves)

def turn_counter(arm):
    go_home(arm)
    moves = []
    moves.append(positions["turn_pounce"])
    moves.append(positions["turn_insert"])
    moves.append(positions["turn_counter"])
    moves.append(positions["turn_pounce_counter"])
    make_moves(arm, moves)
