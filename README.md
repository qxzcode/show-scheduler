# Show Scheduler

**Try it out here:** https://qxzcode.github.io/show-scheduler/

A browser-based tool for building schedules for dance shows, created for a club at RIT. Given a collection of routines to be performed, with some performers participating in multiple routines, it finds a routine order that:
- Minimizes the number of performers with back-to-back routines (no time to change costume).
- Keeps intermission near the middle of the show.
- Respects other custom constraints, such as putting a specific routine directly after another specific routine.

The tool is a single static webpage; everything runs locally in your browser.

## How it works

The core schedule optimization algorithm is implemented in Rust (see [`optimize.rs`](rust/src/optimize.rs)) and compiled to WebAssembly. It uses hill-climbing local search with random restarts and optimizes a sequence of lexicographic objectives. After the user sets up their scenario, the optimizer runs in a background worker and streams solutions (concrete schedules) to the UI as it finds improvements.

I experimented extensively with more "sophisticated" discrete optimization solvers, including CP-SAT, Hexaly, and SCIP – but ultimately my pure-Rust local search implementation is both faster to find good solutions (fast enough to interactively play with the parameters in the UI!) and also simpler to embed in a static page.

## Running locally

Make sure you have [just](https://github.com/casey/just), [bun](https://bun.sh), and [wasm-pack](https://wasm-bindgen.github.io/wasm-pack/) installed.

```sh
just dev      # install deps, build wasm, and start dev server with hot reload
just build    # production build (wasm + vite + prerender) → dist/
just preview  # production build + start preview server
```
