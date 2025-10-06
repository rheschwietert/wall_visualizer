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
- Install dependencies with:

```bash
pip install -r requirements.txt
```

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
   
Order of build modes:

Horizontal Build: Simple course-by-course placement.

Smart Build: Optimized to minimize platform drives and lifts.

---

## Controls

Press ENTER → Place next brick.

Type q and press ENTER → Fill the wall instantly and continue.

