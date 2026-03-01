# AI Pathfinder - Uninformed Search Algorithms

## 📌 Assignment Details
- **Course:** AI 2002 - Artificial Intelligence (Spring 2026)
- **Assignment:** Question 7 - Uninformed Search in Grid Environment
- **Submitted by:** 
  - Noor Ul Huda (24F-0734)
  - Muhammad Ammar (24F-0716)


## 🎯 Description
This project implements 6 uninformed search algorithms in a 10×10 grid environment. The goal is to navigate from Start (S) at (0,0) to Target (T) at (9,9) while avoiding static walls.

### Algorithms Implemented:
1. **BFS** (Breadth-First Search)
2. **DFS** (Depth-First Search)
3. **UCS** (Uniform-Cost Search)
4. **DLS** (Depth-Limited Search)
5. **IDDFS** (Iterative Deepening DFS)
6. **Bidirectional Search**

## 🎮 Features
- ✅ Step-by-step visualization
- ✅ 8-directional movement (including diagonals)
- ✅ Clockwise expansion order
- ✅ Static walls (black cells)
- ✅ Real-time statistics (Explored & Frontier counts)
- ✅ Keyboard controls
- ✅ GUI window titled "GOOD PERFORMANCE TIME APP"

## 🎨 Color Coding
| Color | Meaning |
|-------|---------|
| 🟩 Green | Start Point (0,0) |
| 🟥 Red | Target Point (9,9) |
| ⬛ Black | Static Walls |
| 🟨 Yellow | Frontier Nodes (to be explored) |
| 🟦 Light Blue | Explored Nodes |
| 🟩 Cyan | Final Path |
| ⬜ White | Empty Cells |

## 🎮 Controls
| Key | Function |
|-----|----------|
| **SPACE** | Start the selected search algorithm |
| **R** | Reset the grid |
| **←** | Switch to previous algorithm |
| **→** | Switch to next algorithm |

## 🔧 Requirements
- Python 3.14 or higher
- pygame-ce (Community Edition)

## 📥 Installation

1. **Clone the repository**
```bash
git clone https://github.com/Moor-341/-AI-A1.git
cd -AI-A1
