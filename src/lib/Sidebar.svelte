<script lang="ts">
    interface Props {
        fileName: string;
        numSlots: number;
        status: string;
        score: string;
        running: boolean;
        dragOver: boolean;
        onFile: (file: File) => void;
        onDragOver: () => void;
        onDragLeave: () => void;
    }

    let {
        fileName,
        numSlots = $bindable(),
        status,
        score,
        running,
        dragOver,
        onFile,
        onDragOver,
        onDragLeave,
    }: Props = $props();

    function onFileInput(e: Event) {
        const file = (e.target as HTMLInputElement).files?.[0];
        if (file) onFile(file);
    }

    function onDrop(e: DragEvent) {
        e.preventDefault();
        const file = e.dataTransfer?.files?.[0];
        if (file) onFile(file);
    }
</script>

<aside class="sidebar">
    <div class="sidebar-logo">Show Scheduler</div>

    <section class="sidebar-section">
        <!-- svelte-ignore a11y_no_static_element_interactions -->
        <!-- svelte-ignore a11y_label_has_associated_control -->
        <label class="field-label">CSV of routines</label>
        <div
            class={["drop-zone", dragOver && "drag-over", fileName && "has-file"].filter(Boolean).join(" ")}
            role="button"
            tabindex="0"
            ondragover={(e) => { e.preventDefault(); onDragOver(); }}
            ondragleave={onDragLeave}
            ondrop={onDrop}
            onclick={() => document.getElementById("csv-file-input")?.click()}
            onkeydown={(e) => (e.key === "Enter" || e.key === " ") && document.getElementById("csv-file-input")?.click()}
        >
            {#if fileName}
                <span class="file-name">{fileName}</span>
                <span class="file-hint">Click or drop to replace</span>
            {:else}
                <span class="drop-prompt">Drop CSV or <span class="browse-link">browse</span></span>
            {/if}
        </div>
        <input id="csv-file-input" type="file" accept=".csv,text/csv,text/plain" style="display:none" onchange={onFileInput} />
        <ul class="csv-hints">
            <li><b>Header row:</b> routine names</li>
            <li><b>Cells below each header:</b> performers in that routine, one per cell</li>
        </ul>
    </section>

    <section class="sidebar-section">
        <label class="field-label" for="num-slots">Show slots (incl. intermission)</label>
        <input class="number-input" type="number" id="num-slots" min="1" bind:value={numSlots} />
    </section>

    {#if status}
        <section class="sidebar-section">
            <p class="status-text" class:running>{status}</p>
        </section>
    {/if}

    {#if score}
        <section class="sidebar-section score-section">
            {#each score.split(", ") as part}
                <p class="score-line">{part}</p>
            {/each}
        </section>
    {/if}
</aside>

<style>
    .sidebar {
        width: var(--sidebar-width);
        min-width: var(--sidebar-width);
        background: var(--color-panel);
        border-right: 1px solid var(--color-border);
        padding: 1.25rem 1rem;
        display: flex;
        flex-direction: column;
        gap: 0.25rem;
        overflow-y: auto;
    }

    .sidebar-logo {
        font-size: 1.1rem;
        font-weight: 700;
        color: var(--color-accent-light);
        letter-spacing: 0.02em;
        margin-bottom: 1rem;
    }

    .sidebar-section {
        margin-bottom: 1rem;
    }

    .field-label {
        display: block;
        font-size: 0.75rem;
        font-weight: 600;
        text-transform: uppercase;
        letter-spacing: 0.06em;
        color: var(--color-text-muted);
        margin-bottom: 0.4rem;
    }

    .drop-zone {
        border: 1px dashed var(--color-border);
        border-radius: var(--border-radius-sm);
        padding: 0.75rem;
        text-align: center;
        cursor: pointer;
        color: var(--color-text-faint);
        font-size: 0.85rem;
        transition: border-color 0.15s, background 0.15s;
        user-select: none;
    }

    .drop-zone:hover,
    .drop-zone:focus {
        border-color: var(--color-accent);
        outline: none;
    }

    .drop-zone.drag-over {
        border-color: var(--color-accent);
        background: var(--color-accent-dim);
    }

    .drop-zone.has-file {
        border-style: solid;
        border-color: var(--color-accent);
        background: var(--color-accent-dim);
    }

    .file-name {
        display: block;
        font-family: monospace;
        font-size: 0.8rem;
        font-weight: 700;
        color: var(--color-accent-light);
        word-break: break-all;
    }

    .file-hint {
        display: block;
        font-size: 0.75rem;
        color: var(--color-text-faint);
        margin-top: 0.2rem;
    }

    .csv-hints {
        margin: 0.5rem 0 0 0;
        padding: 0;
        list-style: none;
        font-size: 0.75rem;
        color: var(--color-text-faint);
        line-height: 1.6;
    }

    .csv-hints li {
        margin: 0;
    }

    .browse-link {
        color: var(--color-accent-light);
        text-decoration: underline;
    }

    .number-input {
        width: 100%;
        padding: 0.4rem 0.5rem;
        background: var(--color-surface);
        border: 1px solid var(--color-border);
        border-radius: var(--border-radius-sm);
        color: var(--color-text);
        font-size: 0.9rem;
    }

    .number-input:focus {
        outline: none;
        border-color: var(--color-accent);
    }

    .status-text {
        margin: 0;
        font-size: 0.85rem;
        color: var(--color-text-muted);
        font-style: italic;
    }

    .status-text.running {
        color: var(--color-accent-light);
    }

    .score-section {
        background: var(--color-surface);
        border: 1px solid var(--color-border);
        border-radius: var(--border-radius-sm);
        padding: 0.6rem 0.75rem;
    }

    .score-line {
        margin: 0;
        font-size: 0.8rem;
        color: var(--color-text-muted);
        line-height: 1.6;
    }
</style>
