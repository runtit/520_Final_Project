#ifndef __BLUE_ROBOT_H
#define __BLUE_ROBOT_H

#include "enviro.h"
#include <cmath>
#include <map>
#include <utility>

using namespace enviro;

static inline double br_wrap(double a) {
    while (a >  M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
}
static inline double br_clamp(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// ─────────────────────────────────────────────────────────────────────────────
// BlueRobotController  –  mirror of RedRobotController
//
//  SWEEP   Move right.  No PUSH detection (prevents deadlock).
//  RETURN  Navigate to (target_bx-40, target_by).  → PUSH.
//  PUSH    Deliberate aligned push.  Always exits to RETURN.
//  ESCAPE  Stuck handler with teleport rescue.
// ─────────────────────────────────────────────────────────────────────────────
class BlueRobotController : public Process, public AgentInterface {

    static constexpr double SWEEP_SPEED  = 6.0;
    static constexpr double PUSH_SPEED   = 5.0;
    static constexpr double RET_SPEED    = 6.0;
    static constexpr double ESC_SPEED    = 6.0;
    static constexpr double PUSH_LOSE    = 85.0;
    static constexpr double MAX_Y_LEAN   = 0.7;
    static constexpr double LEAN_GAIN    = 0.015;
    static constexpr double RET_MAX_DEG  = 60.0 * M_PI / 180.0;
    static constexpr int    STUCK_LIMIT  = 35;
    static constexpr int    ESCAPE_TICKS = 80;
    static constexpr int    PUSH_TIMEOUT = 150;
    static constexpr int    RESCUE_AFTER = 3;

public:
    BlueRobotController() : Process(), AgentInterface() {}

    // Initialise FSM state and register all event watchers.
    void init() {
        _game_over    = false;
        _state        = SWEEP;
        _stuck_timer  = 0;
        _escape_timer = 0;
        _push_timer   = 0;
        _escape_count = 0;
        _last_x = _last_y = 0.0;

        // Build a live map of block positions; ignore off-screen scored blocks.
        watch("block_at", [this](Event& e) {
            int bid = e.value()["id"];
            double bx = e.value()["x"], by = e.value()["y"];
            if (std::abs(bx) < 290.0 && std::abs(by) < 180.0)
                _blocks[bid] = { bx, by };
        });
        // Remove a block from the map when it is scored and leaves the field.
        watch("block_removed", [this](Event& e) {
            _blocks.erase((int)e.value()["id"]);
        });
        // Clear the block map when all blocks are re-scattered (game reset).
        watch("scatter_blocks", [this](Event& e) {
            _blocks.clear();
            _escape_count = 0;
        });
        // Stop moving when the round ends.
        watch("game_over", [this](Event& e) {
            _game_over = true;
            track_velocity(0, 0, 50, 800);
        });
        // Handle the Reset button: return to start position and restart FSM.
        watch("button_click", [this](Event& e) {
            if (e.value()["value"] != "reset_game") return;
            _game_over    = false;
            _state        = SWEEP;
            _stuck_timer  = 0;
            _escape_timer = 0;
            _push_timer   = 0;
            _escape_count = 0;
            _blocks.clear();
            teleport(-200.0, 40.0, 0.0);
        });
    }

    // Reset game-over flag when the simulation starts.
    void start() { _game_over = false; _escape_count = 0; }

    void update() {
        if (_game_over) { track_velocity(0, 0, 50, 800); return; }
        auto   p = position();
        double x = p.x, y = p.y;

        // ── Stuck detection ───────────────────────────────────────────────────
        double moved = (x - _last_x)*(x - _last_x) + (y - _last_y)*(y - _last_y);
        _last_x = x;  _last_y = y;
        if (moved < 0.3 && _state != ESCAPE) {
            if (++_stuck_timer >= STUCK_LIMIT) {
                _state = ESCAPE;  _escape_timer = 0;  _stuck_timer = 0;
                _escape_count++;
            }
        } else { _stuck_timer = 0; }

        switch (_state) {

        // ── SWEEP ─────────────────────────────────────────────────────────────
        case SWEEP: {
            double ty      = target_y(x, y);
            double lean    = br_clamp(LEAN_GAIN * (ty - y), -MAX_Y_LEAN, MAX_Y_LEAN);
            double desired = std::atan2(lean, 1.0);
            double err     = br_wrap(desired - angle());
            double w       = br_clamp(3.0 * err, -3.5, 3.5);
            track_velocity(SWEEP_SPEED, w, 15, 500);
            if (x > 290.0) _state = RETURN;
            break;
        }

        // ── RETURN ────────────────────────────────────────────────────────────
        case RETURN: {
            _escape_count = 0;
            auto [tbx, tby] = target_block();
            double dock_offset = (tbx > 220.0) ? 60.0 : 40.0;
            double tx = br_clamp(tbx - dock_offset, -180.0, 180.0);

            double dx = tx - x;
            double dy = tby - y;
            double raw     = std::atan2(dy, std::min(dx, -5.0));
            double dev     = br_wrap(raw - M_PI);
            double desired = M_PI + br_clamp(dev, -RET_MAX_DEG, RET_MAX_DEG);
            double err     = br_wrap(desired - angle());
            double w       = br_clamp(3.0 * err, -3.5, 3.5);
            double v       = RET_SPEED * std::max(0.4, std::cos(err));
            track_velocity(v, w, 15, 500);

            if (std::hypot(dx, dy) < 35.0 || x <= tx) {
                _state = PUSH;  _push_timer = 0;
            }
            break;
        }

        // ── PUSH ──────────────────────────────────────────────────────────────
        case PUSH: {
            auto [pbx, pby] = front_block(x, y);
            bool valid = (pbx > -1e17);
            double dist = valid ? std::hypot(pbx - x, pby - y) : 1e18;
            ++_push_timer;

            if (!valid || dist > PUSH_LOSE || _push_timer > PUSH_TIMEOUT) {
                _state = RETURN;  break;
            }
            if (x > 290.0) { _state = RETURN;  break; }
            // Jammed against top/bottom wall → abort, reposition
            if (std::abs(y) > 160.0) { _state = ESCAPE;  _escape_timer = 0;  _escape_count++;  break; }

            double y_err = pby - y;
            double lean, desired, err, w, v;

            bool last_mile = (pbx > 220.0);   // block close to blue goal

            if (dist < 28.0 || last_mile) {
                lean    = br_clamp(0.02 * y_err, -0.3, 0.3);
                desired = std::atan2(lean, 1.0);
                err     = br_wrap(desired - angle());
                w       = br_clamp(2.5 * err, -2.5, 2.5);
                v       = PUSH_SPEED;
            } else {
                lean    = br_clamp(0.06 * y_err, -1.0, 1.0);
                desired = std::atan2(lean, 1.0);
                err     = br_wrap(desired - angle());
                w       = br_clamp(3.5 * err, -3.5, 3.5);
                v       = PUSH_SPEED * std::max(0.35, 1.0 - 0.4 * std::abs(err));
            }
            track_velocity(v, w, 15, 500);
            break;
        }

        // ── ESCAPE ────────────────────────────────────────────────────────────
        case ESCAPE: {
            if (_escape_count >= RESCUE_AFTER) {
                teleport(-120.0, 130.0, 0.0);
                _escape_count = 0;  _state = SWEEP;
                break;
            }
            // Escape target: drive AWAY from the cross obstacle at (0,0).
            double ex, ey;
            const bool near_cross = (std::abs(x) < 130.0 && std::abs(y) < 130.0);
            if (near_cross) {
                double mag = std::hypot(x, y);
                if (mag < 8.0) {
                    ex = 180.0;  ey = (id() % 2 == 0) ? 130.0 : -130.0;
                } else {
                    ex = br_clamp(x + 200.0 * x / mag, -270.0, 270.0);
                    ey = br_clamp(y + 200.0 * y / mag, -170.0, 170.0);
                }
            } else {
                // Wall-corner escape: drive toward arena centre row (y=0),
                // and step away from whichever side wall we're pressed against.
                ey = 0.0;
                ex = (x > 150.0) ? br_clamp(x - 120.0, -200.0, 200.0)  // pull away from right wall
                                 : 200.0;                                // normal safe zone
            }
            double desired = std::atan2(ey - y, ex - x);
            double err     = br_wrap(desired - angle());
            double w       = br_clamp(3.5 * err, -4.0, 4.0);
            double v       = ESC_SPEED * std::max(0.4, std::cos(err));
            track_velocity(v, w, 15, 500);
            if (++_escape_timer >= ESCAPE_TICKS) {
                _state = RETURN;  _stuck_timer = 0;
            }
            break;
        }
        }
    }

    void stop() {}

private:
    std::pair<double,double> target_block() {
        double max_bx = -1e18, max_by = 0.0;
        for (auto& [id, pos] : _blocks)
            if (pos.first > max_bx) { max_bx = pos.first; max_by = pos.second; }
        return max_bx > -1e18 ? std::make_pair(max_bx, max_by)
                               : std::make_pair(-30.0, 0.0);
    }

    // Block directly ahead (to the right) and within y-range.
    std::pair<double,double> front_block(double rx, double ry) {
        double best = 1e18, bx = -1e18, by = 0.0;
        for (auto& [id, pos] : _blocks) {
            if (pos.first >= rx - 15.0 &&
                    std::abs(pos.second - ry) < 80.0) {
                double d = std::hypot(pos.first - rx, pos.second - ry);
                if (d < best) { best = d; bx = pos.first; by = pos.second; }
            }
        }
        return { bx, by };
    }

    double target_y(double rx, double ry) {
        if (_blocks.empty()) return 0.0;
        double best = 1e18, ty = 0.0;
        for (auto& [id, pos] : _blocks) {
            bool   ahead = (pos.first >= rx - 30.0);
            double score = std::abs(pos.second - ry) + (ahead ? 0.0 : 800.0);
            if (score < best) { best = score; ty = pos.second; }
        }
        return ty;
    }

    bool   _game_over;              // true after "game_over" event; halts update()
    enum State { SWEEP, PUSH, RETURN, ESCAPE };
    State  _state;                  // current FSM state
    int    _stuck_timer;            // ticks since last meaningful movement
    int    _escape_timer;           // ticks spent in ESCAPE state
    int    _push_timer;             // ticks spent in PUSH state (timeout guard)
    int    _escape_count;           // consecutive escapes; triggers teleport rescue
    double _last_x, _last_y;        // position last tick, used by stuck detection
    std::map<int, std::pair<double,double>> _blocks; // live map: block id → (x, y)
};

// Agent container – wraps BlueRobotController into an Enviro Agent.
class BlueRobot : public Agent {
public:
    BlueRobot(json spec, World& world) : Agent(spec, world) { add_process(c); }
private:
    BlueRobotController c;
};

DECLARE_INTERFACE(BlueRobot)
#endif
