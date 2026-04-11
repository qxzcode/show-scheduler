<script lang="ts">
    import init, { parse_csv, optimize } from '$lib/wasm/show_scheduler.js';

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

    const wasmInit = init();

    let csvInput = $state('');
    let numSlots = $state(32);
    let numIter = $state(500);
    let status = $state('');
    let score = $state('');
    let slots = $state<SlotResult[]>([]);
    let running = $state(false);

    async function run() {
        if (!csvInput.trim()) {
            status = 'Please paste a CSV first.';
            return;
        }

        running = true;
        status = `Running ${numIter} iterations… (browser may be unresponsive for a moment)`;
        score = '';
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
            status = 'Done.';
        } catch (err) {
            status = `Error: ${err}`;
        } finally {
            running = false;
        }
    }
</script>

<svelte:head>
    <title>Show Scheduler</title>
</svelte:head>

<main>
    <h1>Show Scheduler</h1>

    <div class="form-row">
        <label for="csv-input">
            Routine CSV (header = routine names, rows = dancer names per column):
        </label>
        <textarea id="csv-input" rows="10" spellcheck="false" placeholder="Paste CSV here…" bind:value={csvInput}></textarea>
    </div>

    <div class="form-row">
        <label for="num-slots">Number of show slots (including intermission):</label>
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
                    {@const isIntermission = slot.routines.length === 1 && slot.routines[0] === '[Intermission]'}
                    {@const hasConflict = slot.dist1_conflicts.length > 0 || slot.dist2_conflicts.length > 0}
                    <tr class:intermission={isIntermission} class:has-conflict={hasConflict}>
                        <td>{slot.slot_number}</td>
                        <td>{slot.routines.join(' + ')}</td>
                        <td><span class="conflict-names">{slot.dist1_conflicts.join(', ')}</span></td>
                        <td><span class="conflict-names">{slot.dist2_conflicts.join(', ')}</span></td>
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

    textarea {
        width: 100%;
        font-family: monospace;
        font-size: 0.85rem;
        box-sizing: border-box;
    }

    input[type='number'] {
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

    :global(.has-conflict td) {
        background: #fff0f0 !important;
    }

    .conflict-names {
        font-size: 0.8rem;
        color: #c00;
    }
</style>
