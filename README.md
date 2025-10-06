## Wall Visualizer

This project simulates building a brick wall with different stacking strategies and bonds.  
It’s a visualization tool that demonstrates:
- Horizontal stacking (row-by-row building).
- Smart stacking (increased efficiency in stacking).
- Simple ASCII rendering of the wall as it is being built.
- Robot reach constraints and stride-based planning.

---

## Requirements

- Python 3.9

---

## Features
- **Brick, Wall & Robot Models** – Define brick dimensions, wall size and robot reach dimensions.
- **Bond System** – Choose at runtime between **Stretcher** and **Wild (Wildverband)**; easy to extend with more.
- **Horizontal Build** – Builds row by row. 
- **Smart Build** – Simulates how a robot would build in minimizing platform drives and lifts.
- **Visualization** – ASCII drawing of the wall updates as you build.

---

##  How to Run

   ```bash
   git clone https://github.com/rheschwietert/wall_visualizer.git
   cd wall_visualizer
   python wall_builder.py
```
**Choosing a bond**

When the program starts, you'll be prompted:
```bash
Choose bond [wild/stretcher] (default: wild):
```
Type wild or stretcher (or press ENTER for the default). The wall is planned and built for that bond.
At the end of the run you’ll see:
```bash
Switch to the other bond and rebuild? [y/N]:
```
Enter y to instantly re-plan and rebuild the same wall with the other bond.

**Order of build modes**

Horizontal Build: Simple course-by-course placement.

Smart Build: Optimized to minimize platform drives and lifts.

---

## Controls

Press ENTER → Place next brick.

Type q and press ENTER → Fill the wall instantly and continue.

## Key Ideas

- **Course planning per bond** — `plan_course(...)` generates each row using the active bond:
  - `StrecherBond`: alternates half/full starts; fills with full bricks.
  - `WildBond`: alternates `half` / `three_quarter` starts, limits consecutive shapes, and avoids repeating step patterns.

- **Indices & geometry** — Every brick carries:
  - grid indices: `brick_x_i`, `brick_y_i`
  - geometry: `center_x`, `center_y`, `prev_row_length`, `new_row_length`
  - state: `built`, `ready`, and the `windows` it belongs to

- **Windowing (robot reach)** — `sliding_window(...)` tiles the wall into reach-sized windows with overlaps.  
  Bricks are assigned via `assign_bricks_to_windows(...)` and sorted by `(brick_y_i, brick_x_i)`.

- **Build directions per pass** — `reorder_windows(...)` computes a zigzag order and per-window `build_directions` (`left`/`right`) used by the dependency logic.

- **Dependencies (“supporters”)** — A brick becomes `ready` only when:
  - it’s inside the *current* window,
  - it’s in the first course **or**
  - it has valid supporters below (plus edge allowances).  
  Implemented in `find_supporters(...)` and checked by `meets_build_requirements(...)`.

- **Run-length limits (Wild)** — To keep visual randomness:
  - Counters `n_cons_halves` and `n_cons_full` track consecutive `half` and `full` placements (incremented in `plan_course(...)` when two same-shape bricks touch).
  - If `n_cons_halves == 3`, `WildBond.next_brick(...)` forces a `full` next (and resets the half counter).
  - If `n_cons_full == 5`, it forces a `half` next (and resets the full counter).

- **Anti-ladder rule (Wild)** — `check_prev_course(...)` prevents repeating step sizes across courses; after 5 identical steps it forces a row reset and replanning.

- **Head-joint distribution (Wild)** — `check_head_joint(...)` computes how many head joints can be widened.  
  In `plan_course(...)`, a subset of joints may become `*_big` to finish the course cleanly (“stootvoegen”), shifting bricks forward accordingly.

## Extending

Add new bonds by subclassing Bond and implementing:

starts_with_half(course_i)

start_brick(course_i, wall)

next_brick(to_fill, brick, wall, course_i)

