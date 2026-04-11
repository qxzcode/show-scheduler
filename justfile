_default:
    just --list

build:
    wasm-pack build --target web --out-dir web/pkg

dev:
    bun run dev --open
