# DRM KeDei 6.2: The Blank White Screen Bug

## The Problem

After converting the KeDei 6.2 display driver from a synchronous pixel push
(which blocked boot for ~80 seconds) to an asynchronous workqueue design,
the display stayed permanently white.  No content was ever shown, despite
the driver loading successfully.

## Root Cause

**DRM's atomic commit ordering calls `pipe_update()` (plane commit) BEFORE
`pipe_enable()` (modeset enable).**  This is by design in
`drm_atomic_helper_commit_tail()` — the standard helper first calls
`commit_planes` and then `commit_modeset_enables`.

In the synchronous version, this was harmless: `pipe_update()` directly
pushed pixels over SPI, blocking the atomic commit thread for ~80 seconds.
By the time `pipe_enable()` ran the init sequence, the bus was idle.  The
init sequence ran cleanly, then subsequent `pipe_update()` calls worked
because the display was already initialized.

In the async version, `pipe_update()` merely recorded a dirty rectangle and
called `schedule_work()`.  The workqueue worker started running
**immediately** — in parallel with the atomic commit thread.  The commit
thread then proceeded to call `pipe_enable()`, which began the R61581
initialization sequence (reset pulse, sleep-out, gamma tables, power
settings, display-on, etc.).

**The result: the worker was firing SPI pixel-data commands (`0x15` prefix)
simultaneously with the init sequence's command/data packets (`0x11`/`0x15`
prefixes).**  Both paths shared the same SPI bus and the same GPIO 8 latch
line.  The interleaved packets corrupted the init sequence — the R61581
received garbled register values and never properly initialized.  The
display stayed in its power-on default state: white.

## Why the Bug Was Hard to Find

1. **The synchronous version worked perfectly** — the 80-second delay was
   the only complaint.  This made it seem like the async conversion was a
   simple refactor, not a fundamental ordering change.

2. **No visible errors** — the SPI transfers all succeeded (no `-EIO`, no
   timeouts).  The corruption was at the LCD controller protocol level,
   invisible to Linux.

3. **The race window was tiny but deterministic** — DRM always commits
   planes before enables in `commit_tail`, so the worker ALWAYS beat the
   init sequence.  It wasn't a flaky race; it failed 100% of the time.

4. **Debug logging initially pointed in the wrong direction** — early logs
   showed `pipe_update` firing and `push_pixels` running, which looked
   correct.  The sheer volume of `pipe_update` logs (~33/sec) obscured the
   critical timing relationship.  It wasn't until the log flood was cleaned
   up and timestamps were examined that the overlap became visible.

## How the Solution Was Found

### Phase 1: Initial debug logging

Added `dev_info` to `pipe_update`, `push_pixels`, and the worker function.
First dmesg output showed hundreds of `pipe_update` lines but only
occasional `push_pixels` — about one per 1.28 seconds (the time for a full
480×320 frame over SPI at 39 MHz with the 3-byte-per-pixel protocol).
This confirmed the async pipeline was functioning but didn't explain why the
screen was white.

### Phase 2: Test fill + pixel dump

Two diagnostics were added:

- A **solid blue test fill** at the end of `pipe_enable()` — to verify the
  init sequence and SPI protocol worked independently of the GEM
  framebuffer path.
- A **one-shot pixel value dump** — first 5 pixels from the GEM buffer, to
  verify the framebuffer contained real data.

The dmesg output from this build was the breakthrough:

```
[4.448554] Enabling display              ← pipe_enable STARTS
[4.448599] push #1: rect=[0,0,480,320]   ← worker runs 45µs later!
[4.448622] pixels[0..4]: 0000 0000 ...   ← valid pixel data
[4.864793] test fill: painting blue       ← init finishes 416ms later
[6.170831] test fill: done
```

The 45-microsecond gap between "Enabling display" and "push #1" proved that
the worker was pushing pixels **during** the init sequence.  The init
sequence takes ~416ms (reset delays, sleep-out, register writes).  The
worker didn't wait — it grabbed its dirty rect and started blasting pixel
data immediately.

The test fill painting blue also didn't show on screen, confirming the init
sequence itself was corrupted by the concurrent SPI traffic.

### Phase 3: The fix

Added a boolean `enabled` flag to the driver state:

- **Starts `false`** (zero-initialized by `devm_drm_dev_alloc`)
- **`pipe_update()` checks it** — returns immediately if `!enabled`,
  silently dropping the premature plane updates
- **Worker also checks it** — belt-and-suspenders guard in the
  spinlock-protected loop
- **Set to `true`** only at the very end of `pipe_enable()`, after the
  entire init sequence completes
- **`pipe_enable()` then queues the first frame push** — it grabs the
  framebuffer from the `plane_state` argument (which DRM passes to the
  enable callback) and schedules the worker with a full-screen dirty rect

This ensures:

1. The init sequence runs on a quiet SPI bus
2. The first pixel push happens immediately after init completes
3. Subsequent `pipe_update()` calls flow through normally
4. `pipe_disable()` clears the flag before cancelling the worker

## The Underlying Design Lesson

The DRM atomic commit model separates "what the hardware state should be"
(plane contents, CRTC mode) from "when to apply it."  The helper
`drm_atomic_helper_commit_tail()` applies planes first because on most
hardware, plane updates are cheap register writes that take effect at the
next vblank.  The modeset enable (which may involve slow PLL lock, panel
power-on sequences, etc.) runs after.

For SPI-based displays with slow initialization sequences, this ordering is
dangerous if `pipe_update()` has side effects on the bus.  The solution is
to **defer all bus activity until `pipe_enable()` signals readiness**, which
is what the `enabled` flag achieves.  This pattern is common in DRM tiny
drivers — for example, `drivers/gpu/drm/tiny/repaper.c` uses a similar
guard.
