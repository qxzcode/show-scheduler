_default:
    just --list

install:
    bun install

check:
    cd rust/ && cargo fmt && cargo clippy

build: install
    bun run build

dev: install
    bun run dev --open

preview: build
    bun run preview --open

# Anonymize a CSV: routine names → "Routine A/B/.../AA/..." and performer names → "Performer 1/2/3/..."
anonymize-csv file:
    #!/usr/bin/env python3
    import csv, os, sys

    file = "{{file}}"

    def col_label(n):
        """0 → A, 1 → B, ..., 25 → Z, 26 → AA, ..."""
        label = ""
        n += 1
        while n > 0:
            n, r = divmod(n - 1, 26)
            label = chr(65 + r) + label
        return label

    with open(file, newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))

    if not rows:
        sys.exit("Error: empty file")

    header, data = rows[0], rows[1:]

    # Map routine names → "Routine A/B/..." (preserve [Intermission] and blank cols)
    routine_map = {}
    routine_idx = 0
    new_header = []
    for col in header:
        if col in ("", "[Intermission]"):
            new_header.append(col)
        else:
            if col not in routine_map:
                routine_map[col] = f"Routine {col_label(routine_idx)}"
                routine_idx += 1
            new_header.append(routine_map[col])

    # Collect performer names in first-appearance order (top-to-bottom, left-to-right)
    seen = set()
    performer_names = []
    for row in data:
        for cell in row:
            name = cell.strip()
            if name and name not in seen:
                performer_names.append(name)
                seen.add(name)

    performer_map = {name: f"Performer {i}" for i, name in enumerate(performer_names, 1)}

    new_data = [
        [performer_map.get(cell.strip(), cell) for cell in row]
        for row in data
    ]

    base, ext = os.path.splitext(file)
    out_file = base + "-anonymized" + ext
    with open(out_file, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(new_header)
        writer.writerows(new_data)

    print(f"Written to {out_file}")
