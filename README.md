## Wall Visualizer

This project simulates building a brick wall with different stacking strategies and robot reachability.  
It’s an educational and visualization tool that demonstrates:
- Horizontal stacking (row-by-row building).
- Smart stacking (increased efficiency in stacking).
- Simple ASCII rendering of the wall as it is being built.
- Robot reach constraints and stride-based planning.

---

## Requirements

- Python 3.9

---

## Features
- **Brick & Wall Models** – Define brick dimensions and wall size.
- **Bond System** – Choose at runtime between **Stretcher** and **Wild (Wildverband)**; easy to extend with more.
- **Horizontal Build** – Build brick by brick by pressing `ENTER`.
- **Smart Build** – Simulates how a robot would build in minimizing platform drives and lifts. Build brick by brick by pressing `ENTER`.
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
Enter y to instantly re-plan and rebuild the exact same wall with the other bond.

**Order of build modes**

Horizontal Build: Simple course-by-course placement.

Smart Build: Optimized to minimize platform drives and lifts.

---

## Controls

Press ENTER → Place next brick.

Type q and press ENTER → Fill the wall instantly and continue.

## Key Ideas

Indices — every brick gets (brick_x_i, brick_y_i) and center_(x,y) for geometry.

Supporters — a brick becomes ready only when its required supporters below are ready (with edge allowances at the ends).

“Big” joints — in WildBond, a few head joints may be widened (*_big) to perfectly finish a course without breaking rules.

## Extending

Add new bonds by subclassing Bond and implementing:

starts_with_half(course_i)

start_brick(course_i, wall)

next_brick(to_fill, brick, wall, course_i)

