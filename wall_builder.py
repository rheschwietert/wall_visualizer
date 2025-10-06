"""
Wall Visualizer

Simulates building a wall with different brick bonds and robot reachability.
Supports:
- Horizontal stacking (simple row by row).
- Smart stacking (windowed reach simulation with dependencies).
"""

# --- Imports___
from dataclasses import dataclass, field
from math import floor, ceil
import os
import random
import copy
random.seed(1)

@dataclass
class Bond:
    """ Base class for bonding logic (how rows start/end and how bricks step).
        Attributes:
            n_cons_halves: Max allowed consecutive half bricks.
            n_cons_full: Max allowed consecutive full bricks.
            build_directions: Per-window pass build direction ("left"|"right").

        Responsibilities:
            - Decide the first brick in a row.
            - Decide the next brick given what is already placed.
            - Provide per-window build directions for plotting/stacking.

        Invariants:
            - `build_directions` length matches the number of windows in the active pass.
            - Step counters only increase for **same** step size (keeps wildverband constraints). """
    n_cons_halves: int = 1
    n_cons_full: int = 1
    build_directions: list = field(default_factory=list)

    def starts_with_half(self, course_i: int) -> bool:
        raise NotImplementedError
    def start_brick(self, course_i, wall):
        raise NotImplementedError
    def next_brick(self, to_fill, brick, wall, course_i):
        raise NotImplementedError
@dataclass
class StrecherBond(Bond):
    """
    Stretcher bond (all stretchers; joints staggered by half a brick).

    Behavior:
        - All inner bricks are 'full'.
        - Start alternates half offsets by row to stagger the vertical head joints."""

    def starts_with_half(self, course_i:int) -> bool:
        return course_i % 2 == 1

    def start_brick(self, course_i, wall):
        return 'half' if course_i % 2 == 1 else 'full' # Odd courses start with half brick

    def next_brick(self, to_fill, brick, wall, course_i):
        if abs(to_fill - brick.full_length) < 1e-3:
            return 'full' #last brick in course is full
        if abs(to_fill - brick.half_length) < 1e-3:
            return 'half' #last brick in course is half
        if to_fill > brick.course_full_length:
            return 'full' # keep filling with full bricks
        return None #finished filling

class WildBond(Bond):
    """Wildverband rules (vrij verband):
          - Start bricks alternate by row (half ↔︎ three_quarter).
          - Limit runs of identical shapes (e.g., max 5 stretchers).
          - Limit consecutive step repeats (avoid visible 'ladders').

        Methods:
            start_brick(row_idx): Enforce alternating starts.
            next_brick(prev_shape, row_idx): Choose next shape respecting run limits.

        Notes:
            - End-of-row completion is handled by planner; this class only chooses shape sequence.
            - Step counters are updated externally in `check_prev_course`."""

    def starts_with_half(self, course_i:int) -> bool:
        return course_i % 2 == 1

    def start_brick(self, course_i, wall):
        return 'half' if course_i % 2 == 1 else 'three_quarter'

    def next_brick(self, to_fill, brick, wall, course_i):
        shapes = ['half', 'full']
        # check consecutive brick shapes
        if self.n_cons_halves == 3:
            if to_fill > brick.course_full_length:
                self.n_cons_halves = 1
                return 'full'
        if self.n_cons_full == 5:
            self.n_cons_full = 1
            return 'half'

        if self.starts_with_half(course_i):
            if to_fill > brick.course_full_length + brick.three_quarter_length:
                return random.choice(shapes) # fill course
            elif to_fill == brick.course_full_length + brick.three_quarter_length:
                return 'full' # only add a full if the row can still be finished
            elif to_fill >= brick.course_half_length + brick.three_quarter_length:
                return 'half' # only add a half if the row can still be finished
        else:
            if to_fill > brick.course_full_length + brick.half_length:
                return random.choice(shapes) # fill course
            elif to_fill == brick.course_full_length + brick.half_length:
                return 'full' # only add a full if the row can still be finished
            elif to_fill >= 2 * brick.course_half_length:
                return 'half' # only add a half if the row can still be finished

        return None

@dataclass
class Brick:
    """Represents a single brick type with geometry and mortar dimensions (mm).
    All brick specs are in mm"""
    full_length: float = 210.0
    half_length: float = 100.0
    three_quarter_length: float = 155.0
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
    """Represents a wall with geometry. All wall specs are in mm"""

    width: float = 2300.0
    height: float = 2000.0

    def n_courses(self, brick: Brick) -> int:
        return floor(self.height / brick.course_height)

    def n_bricks_per_course(self, brick: Brick) -> int:
        return floor(self.width / brick.course_full_length)



@dataclass
class Robot:
    reach_height: float = 1300.0
    reach_width: float = 800.0
    reach_overlap_y: float = 0.0
    reach_overlap_x: float = 0.0

def check_prev_course(course, brick_dict):
    """ Compare this brick's offsets to the previous course to detect
    repeated 'step' patterns that violate the wild bond rules.
    Returns True if the row must be reset. """
    if not course:
        return False

    target_right = next((d for d in course if d.get('new_row_length') > brick_dict['new_row_length']), None)
    target_left = [d for d in course if d.get('prev_row_length') <= brick_dict['prev_row_length']][-1]

    if not target_right or not target_left:
        return False

    # cache step deltas
    length_diff_right = target_right['new_row_length'] - brick_dict['new_row_length']
    length_diff_left = brick_dict['prev_row_length'] - target_left['prev_row_length']
    brick_dict['length_diff_right'] = length_diff_right
    brick_dict['length_diff_left'] = length_diff_left
    brick_dict['n_cons_right_steps'] = 0
    brick_dict['n_cons_left_steps'] = 0
    if length_diff_right == 0.0  or  length_diff_left == 0.0:
        return False

    # count consecutive right-steps
    if target_right.get('length_diff_right') is not None:
        if target_right['length_diff_right'] == brick_dict['length_diff_right']:
            if target_right.get('n_cons_right_steps') is not None:
                brick_dict['n_cons_right_steps'] = target_right['n_cons_right_steps'] + 1
                if brick_dict['n_cons_right_steps'] == 4:
                    return True
            else:
                brick_dict['n_cons_right_steps'] = 1

    # count consecutive left-steps
    if target_left.get('length_diff_left') is not None:
        if target_left['length_diff_left'] == brick_dict['length_diff_left']:
            if target_left.get('n_cons_left_steps') is not None:
                brick_dict['n_cons_left_steps'] = target_left['n_cons_left_steps'] + 1
                if brick_dict['n_cons_left_steps'] == 4:
                    return True
            else:
                brick_dict['n_cons_left_steps'] = 1

    return False


def check_head_joint(wall, brick):
    fill = wall.width - brick.three_quarter_length - brick.course_half_length
    res = fill // brick.course_half_length
    used = res * brick.course_half_length
    extra = fill - used
    res = extra // brick.head_joint
    used2 = res * brick.head_joint
    joint_extra = extra - used2
    if abs(joint_extra) < 1e-3: #no extra distribution for joints
        return 0, 0.0
    return extra / joint_extra, joint_extra


def plan_course(wall, brick, course_i, bond, prev_course):
    """Plan a course of bricks.
        args:
            wall (Wall): Wall dimensions
            brick (Brick): Brick geometry
            course_i (int): Row index (0 = first course)
            bond (Bond): Bond strategy (e.g. stretcher bond)

        returns:
            list of dict: Bricks with placement metadata (position, indices, built flags)"""
    result = []
    filled_length = 0.0
    brick_x_i = 0
    reset = False
    center_y = (course_i) * brick.course_height + brick.height / 2
    if isinstance(bond, WildBond):
        n_joints_max, joint_extra = check_head_joint(wall, brick)

    def add_brick(brick_shape,last_brick_row,):
        """Create and append a brick entry.

            Brick dict schema (keys):
                brick_x_i, brick_y_i : int   # grid indices
                center_x, center_y   : float # center in mm
                shape                : str   # 'half' | 'three_quarter' | 'full' | variants with '_big'
                built, ready         : bool  # build status flags
                new_row_length       : float # cumulative X length from row start to right edge (mm)
                prev_row_length      : float # cumulative X length from row start to left edge (mm)
                windows              : list[int]  # window ids this brick belongs to"""

        nonlocal filled_length, brick_x_i, result
        nonlocal n_joints_max, joint_extra, reset
        if reset:
            brick_x_i = 0
            filled_length = 0.0
            result = []
        reset = False
        prev_length = filled_length
        if brick_shape == 'full':
            length = brick.full_length
        elif brick_shape == 'half':
            length = brick.half_length
        elif brick_shape == 'three_quarter':
            length = brick.three_quarter_length


        filled_length += length
        center_x = filled_length - length / 2

        if not last_brick_row:
            filled_length += brick.head_joint
            center_x += brick.head_joint / 2

        if brick_x_i == 0 and center_x > 200:
            print(center_x)
        if len(result) > 0:
            prev_brick = result[len(result) - 1]
            if prev_brick['shape'] == 'full' == brick_shape:
                bond.n_cons_full += 1
            if prev_brick['shape'] == 'half' == brick_shape:
                bond.n_cons_halves += 1

        brick_dict = {'shape':brick_shape, 'prev_row_length': prev_length,
                      'new_row_length': filled_length, 'built':False,
                      'center_x':center_x, 'center_y': center_y,
                      'brick_x_i':brick_x_i, 'brick_y_i': course_i}

        if isinstance(bond, WildBond):
            reset = check_prev_course(prev_course, brick_dict)
        brick_x_i += 1
        result.append(brick_dict)

    start_brick = bond.start_brick(course_i, wall)
    add_brick(start_brick,False) #add first brick

    # fill until width covered
    while True:
        if isinstance(bond, StrecherBond):
            to_fill = wall.width - filled_length
            nxt = bond.next_brick(to_fill, brick, wall, course_i)
            # check if it is the last brick
            epsilon = 1e-3  # tolerance for length comparisons
            last = abs((wall.width - filled_length) - (brick.full_length if nxt == 'full' else brick.half_length)) < epsilon

        if isinstance(bond, WildBond):
            to_fill = wall.width - filled_length
            nxt = bond.next_brick(to_fill, brick, wall, course_i)
            last = False
            if nxt is None:
                if not bond.starts_with_half(course_i):
                    left_over = to_fill - brick.half_length

                    if left_over < 1e-3:
                        nxt = 'half'
                        last = True
                    # increase several joints to create 'stootvoegen'
                    if abs(to_fill - brick.half_length) == n_joints_max * joint_extra:
                        extra = to_fill - brick.half_length
                        if extra < 0: # reset if the row doesn't work
                            result = []
                            nxt = start_brick
                            filled_length = 0
                            last = False
                            reset = True
                        else:
                            n_joints = int(abs(extra) / joint_extra)
                            if len(result) > n_joints:
                                # shift the bricks due to 'stootvoegen'
                                chosen = random.sample(range(1, len(result)), n_joints)
                                update = lambda d:d.update({'shape': d.get('shape')+'_big', 'center_x': d.get('center_x') + joint_extra,
                                                            'new_row_length': d.get('new_row_length') + joint_extra
                                                            ,'prev_row_length': d.get('prev_row_length') + joint_extra})
                                update_rest = lambda d: d.update({'center_x': d.get('center_x') + joint_extra,
                                                                  'new_row_length': d.get('new_row_length') + joint_extra,
                                                                  'prev_row_length': d.get('prev_row_length') + joint_extra})
                                for i in chosen:
                                    update(result[i])
                                    for d in result[i:]:
                                        update_rest(d)
                                nxt = 'half'
                                last = True

                            else: #reset if the row doesn't work
                                result = []
                                nxt = start_brick
                                filled_length = 0
                                last = False
                                reset = True
                else:
                    left_over = to_fill - brick.three_quarter_length

                    if left_over < 1e-3:
                        nxt = 'three_quarter'
                        last = True

                    # increase several joints to create 'stootvoegen'
                    if abs(to_fill - brick.three_quarter_length) == n_joints_max * joint_extra:
                        extra = to_fill - brick.three_quarter_length
                        if extra < 0:  # reset if the row doesn't work
                            result = []
                            nxt = start_brick
                            filled_length = 0
                            last = False
                            reset = True
                        else:
                            n_joints = int(abs(extra) / joint_extra)
                            if len(result) > n_joints:
                                # shift the bricks due to 'stootvoegen'
                                chosen = random.sample(range(1, len(result)), n_joints)
                                update = lambda d: d.update(
                                    {'shape': d.get('shape') + '_big', 'center_x': d.get('center_x') + joint_extra,
                                     'new_row_length': d.get('new_row_length') + joint_extra,
                                     'prev_row_length': d.get('prev_row_length') + joint_extra})
                                update_rest = lambda d: d.update({'center_x': d.get('center_x') + joint_extra,
                                                                  'new_row_length': d.get('new_row_length') + joint_extra,
                                                                  'prev_row_length': d.get('prev_row_length') + joint_extra})
                                for i in chosen:
                                    update(result[i])
                                    for d in result[i:]:
                                        update_rest(d)
                                nxt = 'three_quarter'
                                last = True

                            else: # reset if the row doesn't work
                                result = []
                                nxt = start_brick
                                filled_length = 0
                                last = False
                                reset = True
            else:
                last = False
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
             Built bricks are '▓', unbuilt bricks are '░'."""
    symbols = {
        'full': ('  ▓▓▓▓▓▓▓▓▓▓', '  ░░░░░░░░░░'),
        'half': ('  ▓▓▓▓', '  ░░░░'),
        'three_quarter': ('  ▓▓▓▓▓▓▓', '  ░░░░░░░'),
        'full_big': ('   ▓▓▓▓▓▓▓▓▓▓', '   ░░░░░░░░░░'),
        'half_big': ('   ▓▓▓▓', '   ░░░░'),
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


def sliding_window(wall, robot, brick, bond):
    """ Partition the wall into robot-sized 'windows' that can be built in steps
       args:
           wall (Wall): Wall dimensions
           robot (Robot): Robot reach dimensions
           brick (Brick): Brick geometry
       returns:
           list of dict: Each dict describes a window (x0,x1,y0,y1,row,column) """

    stride_x = brick.course_full_length # step size x direction
    if isinstance(bond, WildBond):
        stride_x = brick.half_length
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


def assign_bricks_to_windows(wall_design, row_n, sliding_windows, window_design):
    for b in wall_design[f'row {row_n}']:
        b['windows'] = []
        for idx, window in enumerate(sliding_windows):
            if window['x0'] <= b['center_x'] <= window['x1'] and window['y0'] <= b['center_y'] <= window['y1']:
                b['ready'] = False
                b['windows'].append(idx)
                window_design[idx].append(b)
    for i in range(len(sliding_windows)):
        window_design[i].sort(key=lambda b: (b['brick_y_i'], b['brick_x_i']))



def find_supporters(wall_design, brick_dict, wall, bond, brick_idx, win_idx):
    """ Find supporting bricks beneath the given brick.
    Returns list of supporter brick dicts."""
    supporters = []

    x, y = brick_dict['brick_x_i'], brick_dict['brick_y_i']
    filled_length = brick_dict['new_row_length']
    side_supports = []
    left = (x - 1, y)
    right = (x + 1, y)
    below = (x, y - 1)
    xs = [x_ for (x_, yy) in brick_idx.keys() if yy == y]
    if x == 0 or x == max(xs):
            row = wall_design[f'row {y - 1}']
            side_supports.append(next(d for d in row if d.get('new_row_length') >= filled_length))
    else:
        if left in brick_idx:
            side_supports.append(brick_idx[left])
        if right in brick_idx:
            side_supports.append(brick_idx[right])

    if isinstance(bond, WildBond):
        row = wall_design[f'row {y - 1}']
        if bond.build_directions[win_idx] == 'right':
            below = next(d for d in row if d.get('new_row_length') >= filled_length)
            supporters.append(below)
            idx = next(i for i, d in enumerate(row) if d.get('new_row_length') >= filled_length)
            if idx != 0:
                supporters.append(row[idx - 1])
        else:
            filled_length = brick_dict['prev_row_length']
            xs_prev = [x_ for (x_, yy) in brick_idx.keys() if yy == y - 1]
            below = [d for d in row if d.get('prev_row_length') <= filled_length][-1]
            supporters.append(below)
            idx = [i for i, d in enumerate(row) if d.get('prev_row_length') <= filled_length][-1]
            if idx != max(xs_prev):
                supporters.append(row[idx + 1])
            else:
                supporters.append(row[idx - 1])

    else:

        if below in brick_idx:
            supporters.append(brick_idx[below])

        if bond.starts_with_half(y):
            diag = (x - 1, y - 1)
            if diag in brick_idx:
                supporters.append(brick_idx[diag])

        else:
            diag = (x + 1, y - 1)
            if diag in brick_idx:
                supporters.append(brick_idx[diag])

    return supporters, side_supports

def build_brick_idx(wall_design):
    idx = {}

    for row in wall_design.values():
        for b in row:
            idx[(b['brick_x_i'], b['brick_y_i'])] = b
    return idx


def meets_build_requirements(brick_dict, window, wall, bond, wall_design, win_idx, brick_idx):
    """ Check if `brick_dict` can be placed:
        - must be inside the active window
        - first course is always allowed
        - otherwise needs 2 ready supporters beneath
          (with edge-case allowances for half bricks at edges)"""

    if window not in brick_dict['windows']:
        return False


    x, y = brick_dict['brick_x_i'], brick_dict['brick_y_i']

    if brick_dict['brick_y_i'] == 0:
        brick_dict['ready'] = True
        return True #first course can always be placed

    supporters, side_supports = find_supporters(wall_design, brick_dict, wall, bond, brick_idx, win_idx)
    xs = [x_ for (x_, yy) in brick_idx.keys() if yy == y]


    if len(supporters) == 2 and all(s['ready'] for s in supporters) and any(d.get('ready') for d in side_supports):
        brick_dict['ready'] = True
        return True #can be placed if two supporters beneath brick

    if bond.starts_with_half(y) and x == 0 and len(supporters) == 1 and supporters[0]['ready']and side_supports[0]['ready']:
        brick_dict['ready'] = True
        return True #edge case: first half bricks have one supporter

    if not (bond.starts_with_half(y)) and x == max(xs) and len(supporters) == 1 and supporters[0]['ready'] and side_supports[0]['ready']:
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

def smart_stack(wall_design, wall, brick, bond, window_design, sliding_windows, robot):
    """Simulate building the wall with robot windows.

    Iterates over windows in a zigzag order
    At each step, only places bricks that are 'ready' (dependencies satisfied within the current window)
    """
    stride_n = 1
    brick_idx = build_brick_idx(wall_design)

    for i, (win_id, bricks) in enumerate(reorder_windows(window_design, sliding_windows, robot, brick, bond)):
        if bond.build_directions[i] == 'left':
            bricks.sort(key=lambda b: (b["brick_y_i"], -b["brick_x_i"]))
        while True:
            # find the next buildable (and not-yet-built) brick inside this window
            next_brick = None
            for b in bricks:
                if not b.get('built', False) and meets_build_requirements(b, win_id, wall, bond, wall_design, i, brick_idx):
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

def split_row_ids(row_ids, takes, row):
    #stay at the same height but go back and forth depending on the size of the window
    if row % 2 == 1:
        odd_direction = 'left'
        even_direction = 'right'
    else:
        odd_direction = 'right'
        even_direction = 'left'
    if takes == 1:
        return [row_ids], [[odd_direction] * len(row_ids)]
    if takes == 2:
        return [row_ids, row_ids[::-1]], [[odd_direction] * len(row_ids), [even_direction] * len(row_ids[::-1])]
    if takes == 3:
        return [row_ids, row_ids[::-1], row_ids], [[odd_direction] * len(row_ids), [even_direction] * len(row_ids[::-1]), [odd_direction] * len(row_ids)]
    return [row_ids], [[odd_direction] * len(row_ids)]


def calculate_takes(row_height, robot, brick):
    """Calculate how often the robot has to drive the full length of the wall ('takes') for this window row"""
    n_bricks_y = ceil(row_height / brick.course_height)
    n_bricks_x = floor(robot.reach_width / brick.course_full_length) + 1  # half brick added
    height_per_take = n_bricks_x * 2
    return ceil(n_bricks_y / height_per_take)


def reorder_windows(window_design, sliding_windows, robot, brick, bond):
    """Compute the order of windows to process.
        - Windows are grouped row by row (top→bottom)
        - Each row may require multiple 'takes' (left-right passes)"""

    cols = max(w['column'] for w in sliding_windows) + 1
    max_win = max(window_design.keys())
    rows = (max_win // cols) + 1
    ordered_ids = []
    build_dir = []

    for r in range(rows):
        row_ids = list(range(r * cols, min((r + 1) * cols, max_win + 1)))

        row_height = next(
            (w['y1'] - w['y0'] for w in sliding_windows if w['row'] == r and w['column'] == 0),
            0
        )

        takes = calculate_takes(row_height, robot, brick)

        if r % 2 == 1:  # reverse every odd row
            row_ids.reverse()

        # split into 'takes'
        parts, build_dirs = split_row_ids(row_ids, takes, r)
        for part, build_d in zip(parts, build_dirs):
            ordered_ids.extend(part)
            build_dir.extend(build_d)
    bond.build_directions = build_dir
    return [(win_id, window_design[win_id]) for win_id in ordered_ids]


def clear_terminal():
    os.system('cls' if os.name == 'nt' else 'clear')


def check_row_completion(window_design, row, sliding_windows):
    """True if every brick that belongs to this window-row is built"""
    win_idxs_in_row = [i for i, w in enumerate(sliding_windows) if w.get('row') == row]
    bricks_in_row = [item for k in win_idxs_in_row for item in window_design.get(k, [])]
    return all(b.get('built') for b in bricks_in_row)


def make_bond(name: str) -> Bond:
    """Factory: 'wild'|'w' -> WildBond, 'stretcher'|'strecher'|'s' -> StrecherBond."""
    name = (name or "").strip().lower()
    if name in ("wild", "w"):
        return WildBond()
    if name in ("stretcher", "strecher", "s"):
        return StrecherBond()
    raise ValueError(f"Unknown bond: {name!r}")

def run_session(brick: Brick, wall: Wall, robot: Robot, bond: Bond) -> None:
    """Run one full build session (plan → horizontal → smart) for a given bond.

    Uses fresh containers each run so state from one session never leaks into the next.
    """
    # fresh copies (belt & suspenders—keeps per-run state clean)
    brick = copy.deepcopy(brick)
    wall  = copy.deepcopy(wall)
    robot = copy.deepcopy(robot)
    bond  = copy.deepcopy(bond)

    # --- plan windows
    wall_design: dict[str, list[dict]] = {}
    window_design: dict[int, list[dict]] = {}

    sliding_windows = sliding_window(wall, robot, brick, bond)
    for i, _ in enumerate(sliding_windows):
        window_design[i] = []

    # --- plan all courses for the chosen bond
    for n in range(wall.n_courses(brick)):
        prev_course = wall_design[f'row {n-1}'] if n > 0 else None
        wall_design[f'row {n}'] = plan_course(wall, brick, n, bond, prev_course)
        assign_bricks_to_windows(wall_design, n, sliding_windows, window_design)

    # --- horizontal build
    print(f"\n--- HORIZONTAL BUILD ({bond.__class__.__name__}) ---")
    horizontal_stack(wall_design, wall, brick)

    # --- reset to try smart build fairly
    reset_wall(wall_design)

    # --- smart build
    print(f"\n--- SMART BUILD ({bond.__class__.__name__}) ---")
    smart_stack(wall_design, wall, brick, bond, window_design, sliding_windows, robot)


def ask_then_run(brick, wall, robot):
    current = "wild"
    while True:
        run_session(brick, wall, robot, make_bond(current))
        ans = input("Switch to the other bond and rebuild? [y/N]: ").strip().lower()
        if ans not in ("y","yes"):
            break
        current = "stretcher" if current == "wild" else "wild"


def main():
    brick = Brick()
    wall = Wall(2300.0, 2000.0)
    robot = Robot()

    # initial pick
    choice = input("Choose bond [wild/stretcher] (default: wild): ").strip().lower() or "wild"
    current = "wild" if choice not in ("stretcher", "strecher", "s") else "stretcher"

    while True:
        bond = make_bond(current)
        print(f"\n=== RUNNING {current.upper()} BOND ===")
        run_session(brick, wall, robot, bond)

        ans = input("\nSwitch to the other bond and rebuild? [y/N]: ").strip().lower()
        if ans not in ("y", "yes"):
            break
        current = "stretcher" if current == "wild" else "wild"




if __name__ == '__main__':
    main()









