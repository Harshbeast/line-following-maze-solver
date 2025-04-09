# line-following-maze-solver
# 🔍 Black Line Maze Solver Robot (With Encoders)

This Arduino-based robot solves black line mazes using 5 IR sensors, N20 encoder motors, and an L298N motor driver. The robot explores the maze, records the path, optimizes it, and performs a second, faster run using the shortest path.

---

## 🧠 Features

- Line following using 5 IR sensor array
- PID control for smooth line tracking
- Maze exploration with path recording
- Path optimization using pattern matching
- Encoder-based precise turns and movements
- Second run with optimized path

---

## 🛠 Hardware Requirements

| Component               | Quantity |
|------------------------|----------|
| Arduino Uno/Nano       | 1        |
| L298N Motor Driver     | 1        |
| N20 Encoder Motors     | 2        |
| 5-Channel IR Sensor Array | 1      |
| Encoder Wheels         | 2        |
| Push Button (Start)    | 1        |
| Chassis, Wheels, Wires | -        |
| Power Supply (Battery) | 1        |

---

## 🧾 Pin Connections

### 🔌 Motor Driver (L298N)
| Motor | IN1 | IN2 | EN  |
|-------|-----|-----|-----|
| Left  | 7   | 8   | 9   |
| Right | 10  | 12  | 11  |

### 🎛 Encoders
| Encoder | C1 | C2 |
|---------|----|----|
| Left    | 2  | 4  |
| Right   | 3  | 5  |

### 🔦 IR Sensor Array
| Sensor         | Pin |
|----------------|-----|
| Left           | A0  |
| Edge Left      | A1  |
| Center         | A2  |
| Edge Right     | A3  |
| Right          | A4  |

### ⏯ Start Button
- Connected to pin `13` (with internal pull-up)

---

## ⚙️ How It Works

### Phase 1: Maze Exploration
1. Waits for the start button press.
2. Follows the black line using PID.
3. Detects junctions using IR sensors.
4. Logs each move (`L`, `R`, `S`, `U`, `E`) into a `pathTaken` string.

### Path Optimization
- Replaces inefficient path patterns like:
  - `LUL` → `S`
  - `LUS` → `R`
  - `RUL` → `U`
  - `SUL` → `R`

### Phase 2: Optimized Run
1. Waits for second button press.
2. Follows the optimized path for a fast solution.

---

## 🧮 PID Control

- **Kp:** `0.5`
- **Kd:** `0.2`
- **Ki:** `0`
- Adjust `THRESHOLD` based on your IR sensor readings for black vs white detection.

---

## 📦 Code Structure

- `setup()`: Initializes pins, encoders, and serial.
- `loop()`: Handles start, exploration, optimization, and final run.
- `followLine()`: Follows the black line using PID.
- `checkJunction()`: Detects junctions and sets flags.
- `makeDecision()`: Decides path based on flags.
- `optimizePath()`: Simplifies the path.
- `followOptimizedPath()`: Executes optimized path.
- `turnByAngle()`: Turns using encoders.
- `moveDistance()`: Moves fixed distance using encoders.

---

## 📸 Example Sensor Thresholds

> Adjust based on your surface and sensor quality

| Sensor Condition | Value  |
|------------------|--------|
| On White         | ~800–1023 |
| On Black         | ~200–600  |

Set:
```cpp
const int THRESHOLD = 660;
