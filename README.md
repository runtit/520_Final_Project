# Robot Arena: Competitive Block Battle

A two-robot autonomous block-sweeping game built with [Enviro](https://github.com/klavinslab/enviro) and [Elma](https://github.com/klavinslab/elma).  
Two robots — **Red** and **Blue** — compete to push coloured blocks into their respective goal zones before a 120-second timer runs out.

---

## Overview

| Feature | Details |
|---|---|
| Robots | 2 autonomous agents (Red pushes left, Blue pushes right) |
| Blocks | 20 blocks in 4 types (small sky-blue hex, medium gold, medium green octagon, large coral) |
| Goal zones | Red: left wall `x < -280`; Blue: right wall `x > 280` |
| Timer | 120-second countdown; winner shown at game end |
| Reset | "Reset" button restarts the round, re-scatters all blocks |
| Block respawn | If fewer than 15 blocks remain on-field, scored blocks re-enter play |

Each robot runs a four-state finite state machine:

```
SWEEP  →  RETURN  →  PUSH  →  RETURN  (repeat)
   ↘                           ↗
     ESCAPE  (stuck recovery)
```

- **SWEEP** – drive across the arena, nudging any blocks along the way.  
- **RETURN** – navigate behind the leftmost / rightmost target block.  
- **PUSH** – aligned push toward the goal with two sub-phases (approach → contact).  
- **ESCAPE** – triggered when the robot hasn't moved for ~1 s; drives radially away from the obstacle.

---

## Key Challenges & Solutions

| Challenge | Solution |
|---|---|
| Robots pushing blocks the wrong way | Dedicated `SWEEP`/`RETURN` cycle ensuring Red always moves left and Blue always moves right; `PUSH` is only entered from `RETURN` after deliberate positioning |
| Block slipping off during push | Two-phase PUSH: wide y-lean (≤45°) at approach, minimal lean (≤22°) at contact; "last-mile" mode near the goal skips alignment entirely |
| Robot stuck in corners | Wall-boundary exit in PUSH (`|y| > 160`); ESCAPE drives radially away from (0,0) instead of toward it; `teleport` fallback after 3 failed escapes |
| Screen stuttering from 20 simultaneous events | Block position emissions staggered with `id() % 10` offset; fire every 10 ticks instead of 5 |
| Reset button not working | Enviro button format: `"name"` is the event key; event value arrives as full JSON object, requiring `e.value()["value"]` comparison |
| Block respawn accuracy | Replaced unreliable `std::set` window with a simple `_scored_count` counter; respawn fires immediately when `20 - _scored_count < 15` |

---

## Requirements

- [Docker](https://www.docker.com/) with the `klavins/enviro:1.6` image

---

## Installation & Setup

```bash
# 1. Pull the Enviro Docker image (first time only)
docker pull klavins/enviro:1.6

# 2. Clone this repository
git clone <your-repo-url>
cd final_project

# 3. Start the Docker container (mount project as /source)
docker run -p 8765:8765 -v "$(pwd):/source" -it klavins/enviro:1.6 bash
```

---

## Building & Running

Inside the Docker container (the mounted project root is `/source`):

```bash
cd /source

# Compile all agents
make clean && make

# Start the simulation server
enviro
```

Then open a browser and navigate to **http://localhost:8765**.

> **Note:** Always use `make clean && make` after editing any `.h` file.  
> Plain `make` only recompiles when `.cc` files change.

---

## How to Use

| UI Element | Action |
|---|---|
| **Browser window** | Watch the robots compete in real time |
| **HUD (top)** | Live score (`RED N : N BLU`) and countdown timer |
| **Result banner (bottom)** | "RED WINS!" / "BLUE WINS!" / "TIE GAME!" after time expires |
| **Reset button (top-left)** | Restart the round: scores reset to 0, timer resets to 2:00, all blocks re-scatter |

### Block types

| Colour | Shape | Size / notes | Mass | Behaviour |
|---|---|---|---|---|
| Sky-blue | Hexagon | Small (block_sm) | 0.3 | Light, easy to push |
| Gold | Rectangle | Medium (block) | 0.5 | Standard |
| Light green | Octagon | Medium (block_oct) | 0.6 | Standard, rounder shape |
| Coral-orange | Pentagon | Large (block_lg) | 1.0 | Heavy, harder to push |

When fewer than 15 blocks are active, scored blocks automatically re-enter the arena at random positions (avoiding the robot starting zones).

---

## Acknowledgements

- **Enviro / Elma**: [klavinslab](https://github.com/klavinslab/enviro) — multi-agent simulation framework used as the foundation.
- **Chipmunk2D**: physics engine used by Enviro.
- Course materials and examples from *ECE/CSE P 520 – Embedded Systems Programming*, University of Washington.
