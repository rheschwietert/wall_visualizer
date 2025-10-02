## Wall Visualizer

This project simulates building a brick wall with different stacking strategies and robot reachability.  
It’s an educational and visualization tool that demonstrates:
- Horizontal stacking (row-by-row building).
- Smart stacking (using sliding windows and support rules).
- Simple ASCII rendering of the wall as it is being built.
- Robot reach constraints and stride-based planning.

---

## Features
- **Brick & Wall Models** – Define brick dimensions and wall size.
- **Bond System** – Currently supports stretcher bond, easy to extend with other bond types.
- **Horizontal Build** – Build row by row by pressing `ENTER`.
- **Smart Build** – Simulates how a robot would build in minimizing platform drives and lifts.
- **Visualization** – ASCII drawing of the wall updates as you build.

---

##  How to Run
1. Clone the repository:
   ```bash
   git clone https://github.com/YOUR-USERNAME/wall-builder.git
   cd wall-builder
