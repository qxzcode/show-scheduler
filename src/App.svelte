<script lang="ts">
    import init, { optimize, parse_csv } from "$lib/wasm/show_scheduler.js";

    interface SlotResult {
        slot_number: number;
        routines: string[];
        dist1_conflicts: string[];
        dist2_conflicts: string[];
    }

    interface OptimizeResult {
        slots: SlotResult[];
        score: [number, number, number];
    }

    // Guard against SSR context — WASM can't be fetched in Node.
    const wasmInit = typeof window !== "undefined" ? init() : Promise.resolve();

    let csvInput = $state("");
    let numSlots = $state(32);
    let numIter = $state(500);
    let status = $state("");
    let score = $state("");
    let slots = $state<SlotResult[]>([]);
    let running = $state(false);
    let dragOver = $state(false);
    let fileName = $state("");

    function loadFile(file: File) {
        if (
            !file.name.endsWith(".csv") &&
            file.type !== "text/csv" &&
            file.type !== "text/plain"
        ) {
            status = "Please select a CSV file.";
            return;
        }
        const reader = new FileReader();
        reader.onload = (e) => {
            csvInput = (e.target?.result as string) ?? "";
            fileName = file.name;
            status = "";
        };
        reader.readAsText(file);
    }

    function onFileInput(e: Event) {
        const file = (e.target as HTMLInputElement).files?.[0];
        if (file) loadFile(file);
    }

    function onDrop(e: DragEvent) {
        e.preventDefault();
        dragOver = false;
        const file = e.dataTransfer?.files?.[0];
        if (file) loadFile(file);
    }

    async function run() {
        if (!csvInput.trim()) {
            status = "Please select a CSV first.";
            return;
        }

        running = true;
        status = `Running ${numIter} iterations… (browser may be unresponsive for a moment)`;
        score = "";
        slots = [];

        await wasmInit;

        // Defer so the status message renders before WASM blocks the thread.
        await new Promise((r) => setTimeout(r, 0));

        try {
            const routinesJson = parse_csv(csvInput.trim());
            const resultJson = optimize(routinesJson, numSlots, numIter);
            const result: OptimizeResult = JSON.parse(resultJson);

            const [d1, d2, mid] = result.score;
            score = `Score — distance-1 conflicts: ${d1}, distance-2 conflicts: ${d2}, intermission offset from middle: ${mid}`;
            slots = result.slots;
            status = "Done.";
        } catch (err) {
            status = `Error: ${err}`;
        } finally {
            running = false;
        }
    }
</script>

<main>
    <h1>Show Scheduler</h1>

    <div class="form-row">
        <label for="csv-file-input"
            >Routine CSV (header = routine names, rows = dancer names per
            column):</label
        >
        <!-- svelte-ignore a11y_no_static_element_interactions -->
        <div
            class={[
                "drop-zone",
                dragOver ? "drag-over" : "",
                csvInput ? "has-file" : "",
            ]
                .filter(Boolean)
                .join(" ")}
            role="button"
            tabindex="0"
            ondragover={(e) => {
                e.preventDefault();
                dragOver = true;
            }}
            ondragleave={() => {
                dragOver = false;
            }}
            ondrop={onDrop}
            onclick={() => document.getElementById("csv-file-input")?.click()}
            onkeydown={(e) =>
                e.key === "Enter" || e.key === " "
                    ? document.getElementById("csv-file-input")?.click()
                    : null}
        >
            {#if csvInput}
                <span class="file-name">{fileName || "CSV loaded"}</span>
                <span class="file-hint">Click or drop to replace</span>
            {:else}
                <span class="drop-prompt"
                    >Drop CSV file here or <span class="browse-link"
                        >browse</span
                    ></span
                >
            {/if}
        </div>
        <input
            id="csv-file-input"
            type="file"
            accept=".csv,text/csv,text/plain"
            style="display:none"
            onchange={onFileInput}
        />
    </div>

    <div class="form-row">
        <label for="num-slots"
            >Number of show slots (including intermission):</label
        >
        <input type="number" id="num-slots" min="1" bind:value={numSlots} />
    </div>

    <div class="form-row">
        <label for="num-iter">Optimizer iterations:</label>
        <input type="number" id="num-iter" min="1" bind:value={numIter} />
    </div>

    <button disabled={running} onclick={run}>Optimize</button>

    {#if status}
        <p class="status">{status}</p>
    {/if}

    {#if score}
        <p class="score">{score}</p>
    {/if}

    {#if slots.length > 0}
        <table>
            <thead>
                <tr>
                    <th>#</th>
                    <th>Routine(s)</th>
                    <th>Distance-1 conflicts</th>
                    <th>Distance-2 conflicts</th>
                </tr>
            </thead>
            <tbody>
                {#each slots as slot}
                    <tr
                        class={[
                            slot.routines.length === 1 &&
                            slot.routines[0] === "[Intermission]"
                                ? "intermission"
                                : "",
                            slot.dist1_conflicts.length > 0 ||
                            slot.dist2_conflicts.length > 0
                                ? "has-conflict"
                                : "",
                        ]
                            .filter(Boolean)
                            .join(" ")}
                    >
                        <td>{slot.slot_number}</td>
                        <td>{slot.routines.join(" + ")}</td>
                        <td
                            ><span class="conflict-names"
                                >{slot.dist1_conflicts.join(", ")}</span
                            ></td
                        >
                        <td
                            ><span class="conflict-names"
                                >{slot.dist2_conflicts.join(", ")}</span
                            ></td
                        >
                    </tr>
                {/each}
            </tbody>
        </table>
    {/if}
</main>

<style>
    main {
        font-family: sans-serif;
        max-width: 900px;
        margin: 2rem auto;
        padding: 0 1rem;
    }

    h1 {
        margin-bottom: 1.5rem;
    }

    .form-row {
        margin-bottom: 1rem;
    }

    label {
        display: block;
        font-weight: bold;
        margin-bottom: 0.25rem;
    }

    .drop-zone {
        width: 100%;
        box-sizing: border-box;
        border: 2px dashed #aaa;
        border-radius: 6px;
        padding: 2rem 1rem;
        text-align: center;
        cursor: pointer;
        color: #555;
        transition:
            border-color 0.15s,
            background 0.15s;
        user-select: none;
    }

    .drop-zone:hover,
    .drop-zone:focus {
        border-color: #1a6fc4;
        outline: none;
    }

    .drop-zone.drag-over {
        border-color: #1a6fc4;
        background: #eef4fd;
    }

    .drop-zone.has-file {
        border-style: solid;
        border-color: #1a6fc4;
        background: #f5f9ff;
    }

    .drop-prompt {
        font-size: 0.95rem;
    }

    .browse-link {
        color: #1a6fc4;
        text-decoration: underline;
    }

    .file-name {
        display: block;
        font-family: monospace;
        font-size: 0.95rem;
        font-weight: bold;
        color: #1a6fc4;
    }

    .file-hint {
        display: block;
        font-size: 0.8rem;
        color: #888;
        margin-top: 0.25rem;
    }

    input[type="number"] {
        width: 8rem;
        padding: 0.25rem;
    }

    button {
        padding: 0.5rem 1.5rem;
        background: #1a6fc4;
        color: #fff;
        border: none;
        border-radius: 4px;
        cursor: pointer;
        font-size: 1rem;
    }

    button:disabled {
        background: #888;
        cursor: default;
    }

    .status {
        font-style: italic;
        color: #555;
    }

    .score {
        font-weight: bold;
    }

    table {
        border-collapse: collapse;
        width: 100%;
        margin-top: 1rem;
    }

    th,
    td {
        border: 1px solid #ccc;
        padding: 0.4rem 0.7rem;
        text-align: left;
    }

    th {
        background: #f0f0f0;
    }

    tr:nth-child(even) {
        background: #fafafa;
    }

    .intermission {
        background: #fffbe6 !important;
        font-style: italic;
    }

    .has-conflict td {
        background: #fff0f0 !important;
    }

    .conflict-names {
        font-size: 0.8rem;
        color: #c00;
    }
</style>
