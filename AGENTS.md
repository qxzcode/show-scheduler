# Agent Guidelines

## Commands
- After any changes (Rust, TypeScript, Svelte, etc.), run `just check` — it runs `cargo fmt`, `cargo clippy`, and `svelte-check`.
- Run all commands from the project root without a `cd` prefix — the working directory is already correct.
- Do not use absolute paths in Bash commands (e.g. `git remote get-url origin`, not `git -C /home/... remote get-url origin`) — the CWD is already the repo root, and absolute paths trigger extra permission prompts.

## Commits
- Do not include PII in commit messages (no real performer names, even as examples). Use generic descriptions instead.

## Editing
- Preserve all existing comments when editing or rewriting files.
