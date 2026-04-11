_default:
    just --list

install:
    bun install

build: install
    bun run build

dev: install
    bun run dev --open

preview: build
    bun run preview --open
