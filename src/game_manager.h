#ifndef __GAME_MANAGER_H
#define __GAME_MANAGER_H

#include "enviro.h"
#include <string>
#include <chrono>

using namespace enviro;

// ─────────────────────────────────────────────────────────────────────────────
// GameManagerController
//
// Agent is placed at world (0, 0) – arena centre.
//
// SVG coordinate system for decorate() centred on this agent:
//   svg_x =  world_x          (x same direction)
//   svg_y = -world_y          (y is INVERTED: SVG y increases downward)
//
// Key landmarks:
//   Top of arena    world y= 190  → svg_y= -190
//   Bottom of arena world y=-190  → svg_y=  190
//   Red goal   world x ∈ [-340,-280]  → svg x ∈ [-340,-280]
//   Blue goal  world x ∈ [ 280, 340]  → svg x ∈ [ 280, 340]
// ─────────────────────────────────────────────────────────────────────────────
class GameManagerController : public Process, public AgentInterface {

    static constexpr int GAME_DURATION_S    = 120; // round length in seconds
    static constexpr int TOTAL_BLOCKS       = 20;  // fixed number of block agents
    static constexpr int MIN_ACTIVE_BLOCKS  = 15;  // respawn scored blocks when below this

public:
    GameManagerController() : Process(), AgentInterface() {}

    // Initialise all state variables and register event watchers.
    void init() {
        _red_score    = 0;
        _blue_score   = 0;
        _remaining    = GAME_DURATION_S;
        _game_over    = false;
        _started      = false;
        _scored_count = 0;
        _start_time   = std::chrono::steady_clock::now();

        // Increment the relevant team's score and trigger a block respawn
        // if too few blocks remain active on the field.
        watch("block_scored", [this](Event& e) {
            if (_game_over) return;
            std::string team = e.value()["team"];
            if (team == "red")  _red_score++;
            if (team == "blue") _blue_score++;
            _scored_count++;
            draw_hud();
            if (TOTAL_BLOCKS - _scored_count < MIN_ACTIVE_BLOCKS) {
                emit(Event("respawn_blocks", {}));
                _scored_count = 0;
            }
        });

        // Handle the "Reset" button: e.value()["value"] holds the button name.
        // Resets scores, timer, and re-scatters all blocks.
        watch("button_click", [this](Event& e) {
            if (e.value()["value"] != "reset_game") return;
            _red_score    = 0;
            _blue_score   = 0;
            _remaining    = GAME_DURATION_S;
            _game_over    = false;
            _started      = true;
            _scored_count = 0;
            _start_time   = std::chrono::steady_clock::now();
            emit(Event("scatter_blocks", {}));
            draw_hud();
        });
    }

    // Called once when the simulation starts: scatter blocks and start the clock.
    void start() {
        _red_score    = 0;
        _blue_score   = 0;
        _remaining    = GAME_DURATION_S;
        _game_over    = false;
        _started      = true;
        _scored_count = 0;
        _start_time   = std::chrono::steady_clock::now();
        emit(Event("scatter_blocks", {}));
        draw_hud();
    }

    // Called every tick: update the countdown timer and check for game end.
    void update() {
        if (!_started || _game_over) return;
        auto now = std::chrono::steady_clock::now();
        int new_rem = GAME_DURATION_S -
            (int)std::chrono::duration<double>(now - _start_time).count();
        if (new_rem < 0) new_rem = 0;
        if (new_rem != _remaining) {
            _remaining = new_rem;
            draw_hud();
            if (_remaining == 0) {
                _game_over = true;
                draw_hud();
                emit(Event("game_over", {}));
            }
        }
    }

    void stop() {}

private:
    // Render the HUD overlay using SVG via decorate().
    // Draws: goal-zone tints, dashed goal lines, score panel, timer,
    // and (when game over) a winner/tie banner below the arena.
    void draw_hud() {
        std::string r = std::to_string(_red_score);
        std::string b = std::to_string(_blue_score);

        // Timer colour: white → yellow → red
        std::string tcol = "#ffffff";
        if (_remaining <= 30) tcol = "#ffdd00";
        if (_remaining <= 10) tcol = "#ff5533";

        // Format M:SS
        int m = _remaining / 60, s = _remaining % 60;
        std::string tstr = std::to_string(m) + ":" + (s<10?"0":"") + std::to_string(s);

        // ── SVG build ──────────────────────────────────────────────────────────
        // Agent at world (0,0).  svg_y = -world_y.
        //
        // Goal zones span world y ∈ [-190, 190]
        //   → SVG rect y=-190 (top), height=380
        //
        // Score panel near top of arena (world y≈178)
        //   → SVG rect y=-193 (top edge), height=50, text baseline y=-156
        std::string svg =

            // ── Goal zone tinted backgrounds ───────────────────────────────
            "<rect x='-340' y='-190' width='60'  height='380'"
            "  fill='rgba(220,50,50,0.20)' stroke='none'/>"
            "<rect x='280'  y='-190' width='60'  height='380'"
            "  fill='rgba(50,90,220,0.20)' stroke='none'/>"

            // ── Goal boundary dashed lines ──────────────────────────────────
            "<line x1='-280' y1='-190' x2='-280' y2='190'"
            "  stroke='rgba(220,50,50,0.75)' stroke-width='2'"
            "  stroke-dasharray='8,5'/>"
            "<line x1='280'  y1='-190' x2='280'  y2='190'"
            "  stroke='rgba(50,90,220,0.75)' stroke-width='2'"
            "  stroke-dasharray='8,5'/>"

            // ── HUD panel ───────────────────────────────────────────────────
            //   world y=178→svg_y=-178 (text baseline)
            //   panel from svg_y=-196 to -148  (height 48)
            "<rect x='-210' y='-196' width='420' height='48' rx='7'"
            "  fill='rgba(0,0,0,0.65)' stroke='none'/>"

            // Red score
            "<text x='-196' y='-158'"
            "  font-size='28' font-family='monospace' font-weight='bold'"
            "  fill='#ff5555'>RED " + r + "</text>"

            // Colon
            "<text x='-16' y='-158'"
            "  font-size='28' font-family='monospace' fill='white'>:</text>"

            // Blue score
            "<text x='4' y='-158'"
            "  font-size='28' font-family='monospace' font-weight='bold'"
            "  fill='#5599ff'>" + b + " BLU</text>"

            // Divider
            "<line x1='108' y1='-194' x2='108' y2='-150'"
            "  stroke='rgba(255,255,255,0.3)' stroke-width='1'/>"

            // Timer
            "<text x='120' y='-158'"
            "  font-size='28' font-family='monospace' font-weight='bold'"
            "  fill='" + tcol + "'>" + tstr + "</text>";

        // ── Game-over winner banner ───────────────────────────────────────────
        // Placed BELOW the arena bottom wall (svg_y > 190) so remaining blocks
        // in the arena cannot cover it.  Mirrors the HUD which sits above the
        // top wall at svg_y = -196.
        if (_game_over) {
            std::string wtext, wcol;
            if      (_red_score > _blue_score) { wtext="RED WINS!";  wcol="#ff5555"; }
            else if (_blue_score > _red_score) { wtext="BLUE WINS!"; wcol="#5599ff"; }
            else                               { wtext="TIE GAME!";  wcol="#ffdd00"; }

            svg +=
                "<rect x='-185' y='193' width='370' height='68' rx='10'"
                "  fill='rgba(0,0,0,0.88)'"
                "  stroke='rgba(255,255,255,0.35)' stroke-width='2'/>"

                "<text x='0' y='232'"
                "  font-size='42' font-family='sans-serif' font-weight='bold'"
                "  text-anchor='middle' fill='" + wcol + "'>" + wtext + "</text>"

                "<text x='0' y='254'"
                "  font-size='18' font-family='monospace'"
                "  text-anchor='middle' fill='#cccccc'>"
                "Final: RED " + r + "  \xe2\x80\x93  " + b + " BLU</text>";
        }

        decorate(svg);
    }

    int  _red_score;    // blocks Red has scored this round
    int  _blue_score;   // blocks Blue has scored this round
    int  _remaining;    // seconds left on the countdown
    bool _game_over;    // true after timer reaches zero
    bool _started;      // false until start() is called (prevents premature timer)
    int  _scored_count; // how many blocks are currently off-screen (awaiting respawn)
    std::chrono::steady_clock::time_point _start_time; // wall-clock reference for timer
};

// Agent container – wraps GameManagerController into an Enviro Agent.
class GameManager : public Agent {
public:
    GameManager(json spec, World& world) : Agent(spec, world) { add_process(c); }
private:
    GameManagerController c;
};

DECLARE_INTERFACE(GameManager)
#endif
