# Macros v2 — binding the abort, and a macro class

*rev 1, 2026-09-01 · design guide for `G11/grey-deleterium`, sequel to `interruptible-macros-template.md`*

*What this is: your two questions from 1 Sep — (1) stop passing the abort flag into `spin` by hand, (2) a class that owns the task machinery, so you can have several macros — worked as design decisions with names and traps, not as a build order. You said you can mostly see (2) already, so that part is decision points and the two traps you can't see from where you stand; (1) gets the full mechanism since that's the one you asked about. Both shapes were compiled today against stubs with the real PROS signatures, so the skeletons are known-typeable — but nothing here is next. **The current build closes first**: include, delay, then const / new-press / stick-abort, then the home robot test. This file is for after.*

*Done-when for reading this: you can say which option you're building for each part and why, in one sentence each. That's the whole deliverable — a decision, written into this file's margin or your own notes. The build then gets its own done-when before its first line, per the standing rule.*

---

## Part 0 — the principle both parts hang on

Everything below is the single-writer rule scaling up. Today the arbitration is {driver loop, one worker} over one resource. Multiple macros don't change the rule, they multiply the claimants: **ownership is per resource, not global.** Two macros may run at the same instant if and only if they own different resources (a future intake task alongside a drive macro — legitimate). Two macros touching the drivetrain is the judder bug wearing a class. So the question a macro system answers is not "how do I run tasks" — you can already do that — it's "who holds the drivetrain's write token, and how is it claimed and returned."

## Part 1 — stop passing the flag: give the drivetrain an abort source

The mechanism you're missing has a name: **dependency injection at construction.** Instead of every call site handing `spin` the flag, the object is handed it *once*, when the composition root builds it, and keeps an alias as a member. The library still never names `abort_requested` — it stores a pointer it was given. Same boundary as today, moved from per-call to per-object.

```cpp
// drivetrain.hpp
class tank_drivetrain {
    const std::atomic<bool>* abort_source; // pointer, not reference — two reasons below
public:
    tank_drivetrain(std::vector<std::int8_t> left_ports,
                    std::vector<std::int8_t> right_ports,
                    const std::atomic<bool>* abort = nullptr);  // nullptr = "nothing aborts me"
    ...
    tank_drivetrain& spin(double intensity, double timeout_ms);  // two-arg signature returns
};
```

```cpp
// main.cpp — composition root; flags are declared above, so this is legal top-down
tank_drivetrain drivetrain({PORT_L1, PORT_L2, PORT_L3},
                           {PORT_R1, PORT_R2, PORT_R3},
                           &abort_requested);
```

Inside `spin`, the loop clause becomes `... && !(abort_source && *abort_source)` — null means "no source, never aborts," which is why the pointer buys you back the old unconditional behaviour for free.

**Why a pointer member and not a reference member** — both reasons are worth having: a reference member can never be null (no "no abort source" option) and, deeper, it makes the class non-assignable — references can't be reseated, so the compiler deletes `operator=`. Latent until the day something wants to copy or move a drivetrain, then baffling. Search: *reference member assignment operator*.

**What it costs — read this before choosing it.** Binding the source into the object means *every motion on that object respects that one flag in every mode, autonomous included.* The trap: you test driver skills, abort a macro, the flag is left `true` — and next auton run, every motion no-ops instantly, which looks exactly like a dead robot. The guard is one line — clear the flag at the top of `autonomous()` — plus the clearing at claim you already do. Write the guard the same day as the constructor change or it will find you at a competition.

Two decisions left deliberately yours: whether the three-arg `spin` overload survives as an escape hatch or dies (fewer signatures is less to hold; an escape hatch you never use is clutter), and whether the parameter is `set` -able later (`set_abort_source()`) or construction-only (construction-only means no half-configured window — I'd start there). If sources ever multiply — driver flag *or* match-time watchdog *or* wall proximity — the upgrade is storing `std::function<bool()>` instead of a pointer: any yes/no question as an abort source. Park the name; build it when a second source exists.

## Part 2 — the macro class

You said you can see this one, so: the shape, the two traps, and the fork that actually matters.

**What one macro object owns:** its worker task, its own notification (the task's built-in slot — free), its `running` flag, and its routine (`std::function<void()>`, so any callable fits). **What it must not own:** the drivetrain's write token — that belongs to the *resource* and is shared by every claimant, injected into each macro exactly the way Part 1 injects the abort source. Same pattern twice; notice that and both halves of this guide are one idea.

```cpp
class macro {
    std::function<void()> routine;   // before worker, deliberately — see trap 1
    std::atomic<bool>* ownership;    // shared, injected: the resource's write token
    std::atomic<bool>  running{false};
    pros::Task worker;               // declared LAST, also deliberately
public:
    macro(std::function<void()> r, std::atomic<bool>& owned);
    bool start();                    // false = refused, resource already owned
    bool is_running() const;
};
```

**Trap 1 — members initialize in declaration order, and the task starts life immediately.** The worker's lambda captures `this`; the moment the `pros::Task` member constructs, that code is live and can touch the other members. If `worker` were declared first, it could run before `routine` exists. Your worker's first act is parking on `notify_take`, which softens this — it touches nothing until notified, and nobody can notify before the constructor returns — but don't build on that subtlety: declare the task last and the order is safe by construction. Search: *member initialization order*, *`this` escaping the constructor*.

**Trap 2 — claiming the token needs one new atomic idea.** Two macros' `start()` can race in the same instant, and `if (!owned) owned = true;` is a read *then* a write — both can pass the read before either writes. This is the first place your flags genuinely outgrow plain load/store. The fix is the canonical one-liner:

```cpp
bool macro::start() {
    if (ownership->exchange(true)) return false;  // atomically: set true, tell me what it was
    running = true;
    worker.notify();
    return true;
}
```

`exchange` writes and returns the *previous* value as one indivisible operation — if it hands back `true`, someone else already owned the resource and you changed nothing you needed to undo. Name: **test-and-set**, the seed idea under every lock ever built. The worker's epilogue returns the token (`running = false; ownership->store(false);` — token last). Search: *std::atomic exchange*, *test-and-set*.

**The fork that matters: N tasks, or one worker with a routine selector.** N macro objects = N parked tasks = one 32 KB stack each (your `rtos.h` prices it) and the exchange-based refusal above. The alternative: *one* worker task plus "which routine" state set before the notify — either a `std::function` slot the button handler assigns, or the notification's own 32-bit value carrying an index (`notify_ext`, `rtos.hpp:316`, action enum in `rtos.h` — read those doc comments before choosing it). One stack, and drivetrain-exclusivity becomes *structural* — one worker physically can't run two routines at once — instead of enforced by a token. The criterion, and it's Part 0 again: **will two macros ever legitimately run at once on different resources?** If yes (intake + drive), N tasks is real concurrency and the token earns its keep per resource. If no, one-worker-with-selector is simpler, cheaper, and can't have the bug. You don't have a second resource today — but the star drive and its subsystems are coming, so decide on the season's shape, not this week's.

What `opcontrol` becomes either way: a button→macro table and a guard that asks "does anything own the drivetrain" instead of "is my one macro running." The watchdog role doesn't move.

## Names to look up

| Idea | Search / read |
|---|---|
| handing objects their dependencies | dependency injection (constructor injection) |
| pointer vs reference member | reference member; why references make a class non-assignable |
| the task-in-a-class trap | member initialization order; `this` escaping the constructor |
| claiming a shared token | `std::atomic::exchange`; test-and-set |
| routines as values | `std::function` (you've been using it via `pros::Task` all along) |
| index-carrying notifications | `notify_ext`, `rtos.hpp:316` + the action enum in `rtos.h` |
| the whole Part 2 problem | *Game Programming Patterns*, "State" — still the unlocked chapter, still this exact question |

## Keep this

```
ownership is per resource: one write token per shared thing, injected into every claimant
part 1 = part 2 = dependency injection: bind once at construction, library never names the global
pointer member, not reference: nullable, and the class stays assignable
declare the task member last; claim tokens with exchange(true), never check-then-set
the real fork: N tasks (real concurrency, token per resource) vs one worker + selector (exclusive by shape)
```

## Next action

Not this file. The current build's five leftovers, then the home robot test against the template's done-when. This guide's only near-term output is two sentences — which option for Part 1, which for Part 2 — written down when you've slept on it.
