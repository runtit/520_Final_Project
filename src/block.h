#ifndef __BLOCK_H
#define __BLOCK_H

#include "enviro.h"
#include <random>
#include <cmath>

using namespace enviro;

// ─────────────────────────────────────────────────────────────────────────────
// BlockController  –  Process attached to every scoring block agent.
//
// Responsibilities:
//   • Damp residual movement so blocks don't drift indefinitely.
//   • Detect when a block crosses a goal line and emit scoring events.
//   • Broadcast the block's position periodically so robots can track it.
//   • Respawn off-screen blocks to random valid positions on demand.
//
// Events emitted:
//   "block_at"      every 10 ticks while active  → {id, x, y}
//   "block_scored"  when x crosses a goal line   → {team}
//   "block_removed" same moment as block_scored  → {id}
//
// Events watched:
//   "scatter_blocks"  – teleport ALL blocks to random positions (on game reset)
//   "respawn_blocks"  – teleport only SCORED (off-screen) blocks back into play
// ─────────────────────────────────────────────────────────────────────────────
class BlockController : public Process, public AgentInterface {

public:
    BlockController() : Process(), AgentInterface() {}

    // Seed the RNG and register event watchers.
    void init() {
        std::random_device rd;
        _rng.seed(rd());
        _scored     = false;
        // Stagger emission timers across all 20 blocks using the agent's unique
        // id so they don't all fire on the same tick (prevents event bursts).
        _emit_timer = id() % 10;

        // Reset and re-scatter every block (called on game reset).
        watch("scatter_blocks", [this](Event& e) {
            _scored     = false;
            _emit_timer = id() % 10;
            scatter();
        });

        // Respawn only scored (off-screen) blocks back into play.
        // Blocks that are still active on the field are left undisturbed.
        watch("respawn_blocks", [this](Event&) {
            if (!_scored) return;
            _scored     = false;
            _emit_timer = id() % 10;
            scatter();
        });
    }

    void start() {}

    // Called every simulation tick.
    void update() {
        damp_movement();   // bleed off any lingering velocity

        if (_scored) return;   // off-screen blocks do nothing

        double x = position().x;

        // ── Goal zone detection ───────────────────────────────────────────────
        // Red goal: x < -280  |  Blue goal: x > +280
        if (x < -280.0) {
            _scored = true;
            emit(Event("block_scored",  { {"team", "red"} }));
            emit(Event("block_removed", { {"id",   id()}  }));
            teleport(0.0, -350.0, 0.0);   // move off-screen below the arena
            return;
        }
        if (x > 280.0) {
            _scored = true;
            emit(Event("block_scored",  { {"team", "blue"} }));
            emit(Event("block_removed", { {"id",   id()}   }));
            teleport(0.0, -350.0, 0.0);
            return;
        }

        // ── Broadcast position every 10 ticks so robots can find us ──────────
        if (++_emit_timer >= 10) {
            _emit_timer = 0;
            auto p = position();
            emit(Event("block_at", { {"id", id()}, {"x", p.x}, {"y", p.y} }));
        }
    }

    void stop() {}

private:
    // Teleport the block to a random position inside the arena, avoiding the
    // two robot starting zones (radius 40 around (±200, 0)).
    void scatter() {
        std::uniform_real_distribution<double> xd(-230.0,  230.0);
        std::uniform_real_distribution<double> yd(-155.0,  155.0);
        std::uniform_real_distribution<double> td(   0.0, 6.2832);
        double x, y;
        int tries = 0;
        do {
            x = xd(_rng); y = yd(_rng);
            if (++tries > 200) break;
        } while (dist2(x,y,-200,0) < 1600 || dist2(x,y,200,0) < 1600);
        teleport(x, y, td(_rng));
    }

    // Squared Euclidean distance helper (avoids a sqrt).
    static double dist2(double x1,double y1,double x2,double y2) {
        double dx=x1-x2, dy=y1-y2; return dx*dx+dy*dy;
    }

    std::mt19937 _rng;        // Mersenne-Twister RNG for random placement
    bool         _scored;     // true while block is off-screen after scoring
    int          _emit_timer; // counts ticks between block_at broadcasts
};

// Agent container – wraps BlockController into an Enviro Agent.
class Block : public Agent {
public:
    Block(json spec, World& world) : Agent(spec, world) { add_process(c); }
private:
    BlockController c;
};

DECLARE_INTERFACE(Block)
#endif
