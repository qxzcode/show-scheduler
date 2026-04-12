# Show Scheduler

**It's live here:** https://qxzcode.github.io/show-scheduler/

A browser-based tool for finding a good act order for dance shows. It minimizes the number of performers with back-to-back routines (no time to change costume), while keeping intermission near the middle of the show.

The tool is a single static webpage; everything runs locally in your browser.

## How it works

The core schedule optimization algorithm is implemented in Rust and compiled to WASM. It uses hill-climbing local search with random restarts. After the user sets up the problem, the optimizer runs in a background worker and streams solutions (concrete schedules) to the UI as it finds improvements.

I experimented extensively with more "sophisticated" discrete optimization solvers, including CP-SAT, Hexaly, and SCIP – but ultimately my pure-Rust local search implementation is both faster to find good solutions (fast enough to interactively play with the parameters in the UI!) and also simpler to embed in a static page.

## Running locally

Make sure you have [just](https://github.com/casey/just), [bun](https://bun.sh), and [wasm-pack](https://wasm-bindgen.github.io/wasm-pack/) installed.

```sh
just dev      # install deps, build wasm, and start dev server with hot reload
just build    # production build (wasm + vite + prerender)
just preview  # production build + start preview server
```
