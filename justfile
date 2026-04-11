_default:
    just --list

build:
    wasm-pack build --target web --out-dir web/pkg

serve: build
    python3 -m http.server --directory web
