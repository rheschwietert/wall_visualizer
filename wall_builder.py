"""
Wall Visualizer

Simulates building a wall with different brick bonds and robot reachability.
Supports:
- Horizontal stacking (simple row by row).
- Smart stacking (windowed reach simulation with dependencies).
"""

# --- Imports___
from dataclasses import dataclass
from math import floor, ceil
import os


class Bond:
    def start_brick(self, course_i, wall):
        raise NotImplementedError
    def next_brick(self, course_i, wall):
        raise NotImplementedError

class StrecherBond(Bond):
    def start_brick(self, course_i, wall):
        return 'half' if course_i % 2 == 1 else 'full' # Odd courses start with half brick

    def next_brick(self, to_fill, brick):
        if abs(to_fill - brick.full_length) < 1e-3:
            return 'full' #last brick in course is full
        if abs(to_fill - brick.half_length) < 1e-3:
            return 'half' #last brick in course is half
        if to_fill > brick.course_full_length:
            return 'full' # keep filling with full bricks
        return None #finished filling

@dataclass(frozen=True)
class Brick:
    """Represents a single brick type with geometry and mortar dimensions (mm).
    All brick specs are in mm"""
    full_length: float = 210.0
    half_length: float = 100.0
    height: float = 50.0
    head_joint: float = 10.0
    bed_joint: float = 12.5

    @property
    def course_height(self) -> float:
        return self.height + self.bed_joint

    @property
    def course_full_length(self) -> float:
        return self.full_length + self.head_joint

    @property
    def course_half_length(self) -> float:
        return self.half_length + self.head_joint

@dataclass
class Wall:
    # all wall specs in mm
    width: float = 2300.0
    height: float = 2000.0

    def n_courses(self, brick: Brick) -> int:
        '''finds number of courses in wall'''
        return floor(self.height / brick.course_height)

    def n_bricks_per_course(self, brick: Brick) -> int:
        '''returns number of bricks per course'''
        return floor(self.width / brick.course_full_length)

    def starts_with_half(self, course_i:int) -> bool:
        '''checks if the course starts with half a brick'''
        return course_i % 2 == 1


@dataclass
class Robot:
    reach_height: float = 1300.0
    reach_width: float = 800.0
    reach_overlap_y: float = 0.0
    reach_overlap_x: float = 0.0

def plan_course(wall, brick, course_i, bond):
    """Plan a course of bricks.
        args:
            wall (Wall): Wall dimensions
            brick (Brick): Brick geometry
            course_i (int): Row index (0 = first course)
            bond (Bond): Bond strategy (e.g. stretcher bond)

        returns:
            list of dict: Bricks with placement metadata (position, indices, built flags)
        """
    result = []
    filled_length = 0.0
    brick_x_i = 0

    center_y = (course_i) * brick.course_height + brick.height / 2

    def add_brick(brick_shape,last_brick_row):
        nonlocal filled_length, brick_x_i
        prev_length = filled_length
        length = brick.full_length if brick_shape == 'full' else brick.half_length
        filled_length += length
        center_x = filled_length - length / 2

        if not last_brick_row:
            filled_length += brick.head_joint
            center_x += brick.head_joint / 2

        brick_dict = {'shape':brick_shape, 'prev_row_length': prev_length,
                      'new_row_length': filled_length, 'built':False,
                      'center_x':center_x, 'center_y': center_y,
                      'brick_x_i':brick_x_i, 'brick_y_i': course_i}
        brick_x_i += 1
        result.append(brick_dict)

    start_brick = bond.start_brick(course_i, wall)
    add_brick(start_brick,False) #add first brick

    # fill until width covered
    while True:
        to_fill = wall.width - filled_length
        nxt = bond.next_brick(to_fill, brick)
        if nxt is None: break

        #check if it is the last brick
        epsilon = 1e-3 # tolerance for length comparisons
        last = abs((wall.width - filled_length) - (brick.full_length if nxt == 'full' else brick.half_length)) < epsilon
        add_brick(nxt, last)
        if last:
            break
    return result

def draw_wall(wall_design):
    """ Render the current wall state as ASCII art
    args:
        wall_design (dict): Mapping of row_id -> list of brick dicts
    returns:
        str: Multiline string showing the wall.
             Built bricks are '▓', unbuilt bricks are '░'.
    """
    symbols = {
        'full': ('▓▓▓▓ ', '░░░░ '),
        'half': ('▓▓ ', '░░ ')
    }
    rows = []
    for row in wall_design.values():
        row_str = "".join(
            symbols[b['shape']][0] if b['built'] else symbols[b['shape']][1]
            for b in row
        )
        rows.append(row_str)
    return "\n".join(reversed(rows))

def fill_wall(wall_design):
    for row in wall_design.values():
        for b in row:
            b["built"] = True



def sliding_window(wall, robot, brick):
    """ Partition the wall into robot-sized 'windows' that can be built in steps
       args:
           wall (Wall): Wall dimensions
           robot (Robot): Robot reach dimensions
           brick (Brick): Brick geometry
       returns:
           list of dict: Each dict describes a window (x0,x1,y0,y1,row,column) """

    stride_x = brick.course_full_length  # step size x direction
    x_overlap = robot.reach_width - stride_x # overlap between windows in x direction

    cols = ceil((wall.width - robot.reach_width) / stride_x) + 1 # number of window columns
    rows = ceil(wall.height/robot.reach_height) # number of window rows

    y_overlap = rows * robot.reach_height - wall.height # overlap between windows in y direction
    robot.reach_overlap_y = y_overlap
    robot.reach_overlap_x = x_overlap

    windows = []
    for row in range(rows):
        # vertical window bounds
        y0 = row * (robot.reach_height - y_overlap)
        y1 = min(y0 + robot.reach_height, wall.height)

        for col in range(cols):
            # horizontal window bounds
            if col == 0:
                x0 = 0
            else:
                x0 = col * (robot.reach_width - x_overlap)

            if col == cols - 1:
                x1 = wall.width
                x0 = max(0, x1 - robot.reach_width)
            else:
                x1 = min(x0 + robot.reach_width, wall.width)

            windows.append({'row': row,
                            'column':col,
                            'x0':x0, 'x1':x1,
                            'y0':y0, 'y1':y1
                            })

    return windows


def assign_bricks_to_windows(wall_design, row_n, brick, sliding_windows, window_design):
    for b in wall_design[f'row {row_n}']:
        b['windows'] = []
        for i, window in enumerate(sliding_windows):
            if window['x0'] <= b['center_x'] <= window['x1'] and window['y0'] <= b['center_y'] <= window['y1']:
                b['ready'] = False
                b['windows'].append(i)
                window_design[i].append(b)
        if b['windows'] == []:
            print('ah)')
    return None



def find_supporters(brick_idx, x, y, wall):
    """ Find supporting bricks beneath the given brick.
    Returns list of supporter brick dicts."""
    supporters = []
    below = (x, y -1)

    if below in brick_idx:
        supporters.append(brick_idx[below])

    if wall.starts_with_half(y):
        diag = (x - 1, y - 1)
        if diag in brick_idx:
            supporters.append(brick_idx[diag])

    else:
        diag = (x + 1, y - 1)
        if diag in brick_idx:
            supporters.append(brick_idx[diag])

    return supporters

def build_brick_idx(wall_design):
    idx = {}
    for row in wall_design.values():
        for b in row:
            idx[(b['brick_x_i'], b['brick_y_i'])] = b
    return idx

def meets_build_requirements(brick_dict, window, wall, brick, brick_idx):
    """Check if a brick can be built, given window and supporting bricks."""

    if window not in brick_dict['windows']:
        return False

    x, y = brick_dict['brick_x_i'], brick_dict['brick_y_i']

    if y == 0:
        brick_dict['ready'] = True
        return True #first course can always be placed

    supporters = find_supporters(brick_idx, x, y, wall)

    if len(supporters) == 2 and all(s['ready'] for s in supporters):
        brick_dict['ready'] = True
        return True #can be placed if two supporters beneath brick

    if wall.starts_with_half(y) and x == 0 and len(supporters) == 1 and supporters[0]['ready']:
        brick_dict['ready'] = True
        return True #edge case: first half bricks have one supporter

    if not (wall.starts_with_half(y)) and x == wall.n_bricks_per_course(brick) and len(supporters) == 1 and supporters[0]['ready']:
        brick_dict['ready'] = True
        return True #edge case: last half bricks have one supporter

    return False


def horizontal_stack(wall_design, wall, brick):
    """Simulates building the wall course by course (horizontally)."""
    quit = False
    for n in range(wall.n_courses(brick)):
        for m in wall_design[f'row {n}']:
            cmd = input("[ENTER]=next brick | q=fill wall and quit: ").strip().lower()
            if cmd == 'q':
                fill_wall(wall_design)
                quit = True
                clear_terminal()
                print(draw_wall(wall_design))
                break
            m['built'] = True
            clear_terminal()
            print(draw_wall(wall_design))
            print(f'Placed {m}, in row {n}')

        if quit:
            break

    print('Wall completed')


def reset_wall(wall_design):
    for row in wall_design.values():
        for b in row:
            b['built'] = False
            b['ready'] = False

def smart_stack(wall_design, wall, brick, window_design, sliding_windows, robot, brick_idx):
    """Simulate building the wall with robot windows.

    Iterates over windows in a zigzag order
    At each step, only places bricks that are 'ready' (dependencies satisfied within the current window)
    """
    stride_n = 1
    for win_id, bricks in reorder_windows(window_design,sliding_windows, robot, brick):
        while True:
            # find the next buildable (and not-yet-built) brick inside this window
            next_brick = None
            for b in bricks:
                if not b.get('built', False) and meets_build_requirements(b, win_id, wall, brick, brick_idx):
                    next_brick = b
                    break

            # none left to place in this stride → move to next stride
            if next_brick is None:
                break

            cmd = input("press ENTER to place next brick (or 'q' + ENTER to quit): ").strip().lower()
            if cmd == 'q':
                # fill everything and show the final wall, then exit main()
                fill_wall(wall_design)
                clear_terminal()
                print(draw_wall(wall_design))
                print("Wall filled and finished.")
                return

            # place exactly one brick
            next_brick['ready'] = True
            next_brick['built'] = True

            # show ASCII wall after this single placement
            clear_terminal()
            print(draw_wall(wall_design))
            x, y = next_brick['center_x'], next_brick['center_y']
            print(f'placed brick at x:{x} y:{y} in stride: {stride_n}')
        stride_n += 1
    print("Smart build finished")

def split_row_ids(row_ids, takes):
    #stay at the same height but go back and forth depending on the size of the window
    if takes == 1:
        return [row_ids]
    if takes == 2:
        return [row_ids, row_ids[::-1][1:]]
    if takes == 3:
        return [row_ids, row_ids[::-1][1:], row_ids[1:]]
    return [row_ids]


def calculate_takes(row_height, robot, brick, is_last_row, overlap_y):
    """Calculate how often the robot has to drive the full length of the wall ('takes') for this window row"""
    n_bricks_y = ceil(row_height / brick.course_height)
    n_bricks_x = floor(robot.reach_width / brick.course_full_length) + 1  # half brick added
    if  is_last_row:
        n_bricks_y = ceil((row_height - overlap_y) / brick.course_height)
    height_per_take = n_bricks_x * 2
    return ceil(n_bricks_y / height_per_take)


def reorder_windows(window_design, sliding_windows, robot, brick):
    """Compute the order of windows to process.

        - Windows are grouped row by row (top→bottom)
        - Each row may require multiple 'takes' (left-right passes)
        - Odd rows are reversed (right->left)"""

    cols = max(w['column'] for w in sliding_windows) + 1
    max_win = max(window_design.keys())
    rows = (max_win // cols) + 1
    ordered_ids = []

    for r in range(rows):
        row_ids = list(range(r * cols, min((r + 1) * cols, max_win + 1)))

        row_height = next(
            (w['y1'] - w['y0'] for w in sliding_windows if w['row'] == r and w['column'] == 0),
            0
        )

        takes = calculate_takes(
            row_height=row_height,
            robot=robot,
            brick=brick,
            is_last_row=(r == rows - 1),
            overlap_y=robot.reach_overlap_y
        )

        if r % 2 == 1:  # reverse every odd row
            row_ids.reverse()

        # split into 'takes'
        for part in split_row_ids(row_ids, takes):
            ordered_ids.extend(part)

    return [(win_id, window_design[win_id]) for win_id in ordered_ids]


def clear_terminal():
    os.system('cls' if os.name == 'nt' else 'clear')


## DEBUGGING
def plot_wall(wall, sliding_windows, window_design, wall_design, brick, robot):
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches

    fig, ax = plt.subplots()
    rect = patches.Rectangle((0, 0), wall.width, wall.height,
                             linewidth=2, edgecolor='black', facecolor='none')

    ax.add_patch(rect)
    count = 1

    for win_id, bricks in reorder_windows(window_design, sliding_windows, robot, brick):
        print(count)
        count += 1

        w = sliding_windows[win_id]
        brick_idx = build_brick_idx(wall_design)
        x0, x1, y0, y1 = w['x0'], w['x1'], w['y0'], w['y1']
        width = x1 - x0
        height = y1 - y0
        rect = patches.Rectangle((x0, y0), width, height,
                                     linewidth=1, edgecolor='blue', facecolor='none', linestyle='--')
        ax.add_patch(rect)
        for b in bricks:
            x, y = b['center_x'], b['center_y']
            meets_build_requirements(b, win_id, wall, brick, brick_idx)
            ax.plot(x, y, 'o', color='yellow', markersize=4)
            if b['ready']:
                color = 'blue'
                b['built'] = True
            else:
                color = 'red'
                b['built'] = False

            ax.plot(x, y, 'o', color=color, markersize=4)



    ax.set_xlim(-50, wall.width + 50)
    ax.set_ylim(-50, wall.height + 50)

    ax.set_aspect('equal', adjustable='box')
    plt.show()


def main():
    brick = Brick()
    wall = Wall(2300.0, 2000.0)
    robot = Robot()
    bond = StrecherBond()

    wall_design = {}
    window_design = {}
    sliding_windows = sliding_window(wall, robot, brick)
    for i, window in enumerate(sliding_windows):
        window_design[i] = []


    for n in range(wall.n_courses(brick)):
        wall_design[f'row {n}'] = plan_course(wall, brick, n, bond)
        assign_bricks_to_windows(wall_design, n, brick, sliding_windows, window_design)
    brick_idx = build_brick_idx(wall_design)

    # horizontal build
    print("\n--- HORIZONTAL BUILD ---")
    horizontal_stack(wall_design, wall, brick)

    # reset to try smart mode fairly
    reset_wall(wall_design)

    # smart build
    print("\n--- SMART BUILD ---")
    smart_stack(wall_design, wall, brick, window_design, sliding_windows, robot, brick_idx)



    ## DEBUG
    # plot_wall(wall, sliding_windows, window_design, wall_design, brick, robot)


if __name__ == '__main__':
    main()









