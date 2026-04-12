<script lang="ts">
    interface Props {
        fileName: string;
        numSlots: number;
        minSlots: number;
        maxSlots: number;
        intermissionTolerance: number;
        maxIntermissionTolerance: number;
        status: string;
        score: { d1: number; d2: number; mid: number } | null;
        running: boolean;
        dragOver: boolean;
        variant?: 'sidebar' | 'tab';
        onFile: (file: File) => void;
        onDragOver: () => void;
        onDragLeave: () => void;
        onRegenerate?: () => void;
    }

    let {
        fileName,
        numSlots = $bindable(),
        minSlots,
        maxSlots,
        intermissionTolerance = $bindable(),
        maxIntermissionTolerance,
        status,
        score,
        running,
        dragOver,
        variant = 'sidebar',
        onFile,
        onDragOver,
        onDragLeave,
        onRegenerate,
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

<div class={variant === 'sidebar' ? 'sidebar' : 'setup-tab'}>
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
        <label class="field-label" for="num-slots">
            Show slots (incl. intermission)
            {#if fileName}<span class="slot-value">{numSlots}</span>{/if}
        </label>
        <input
            class="slot-slider"
            type="range"
            id="num-slots"
            min={minSlots}
            max={maxSlots}
            step="1"
            disabled={!fileName}
            bind:value={numSlots}
        />
        {#if fileName}
            <div class="slider-bounds">
                <span>{minSlots}</span>
                <span>{maxSlots}</span>
            </div>
        {/if}
    </section>

    <section class="sidebar-section">
        <label class="field-label" for="intermission-tolerance">
            Intermission tolerance
            {#if fileName}<span class="slot-value">{intermissionTolerance}</span>{/if}
        </label>
        <input
            class="slot-slider"
            type="range"
            id="intermission-tolerance"
            min="0"
            max={maxIntermissionTolerance}
            step="1"
            disabled={!fileName}
            bind:value={intermissionTolerance}
        />
        {#if fileName}
            <div class="slider-bounds">
                <span>strict</span>
                <span>loose</span>
            </div>
        {/if}
    </section>

    {#if status}
        <section class="sidebar-section">
            <p class="status-text" class:running>{status}</p>
        </section>
    {/if}

    {#if fileName}
        <section class="sidebar-section score-section">
            <p class="score-label">Performers in adjacent routines</p>
            <p class="score-value" class:score-bad={score && score.d1 > 0}>
                {#if score}{score.d1}{:else}<em class="score-pending">working…</em>{/if}
            </p>
            <p class="score-label">Performers with one-routine turnarounds</p>
            <p class="score-value" class:score-warn={score && score.d2 > 0}>
                {#if score}{score.d2}{:else}<em class="score-pending">working…</em>{/if}
            </p>
            <p class="score-label">Intermission distance from center</p>
            <p class="score-value" class:score-warn={score && score.mid > intermissionTolerance}>
                {#if score}{score.mid === 0 ? "Centered" : `${score.mid} slot${score.mid === 1 ? "" : "s"} off`}{:else}<em class="score-pending">working…</em>{/if}
            </p>
        </section>
    {/if}

    {#if fileName}
        <section class="sidebar-section">
            <button class="regen-btn" onclick={onRegenerate}>
                Generate another schedule
            </button>
        </section>
    {/if}
</div>

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

    /* Rendered as a tab panel on mobile — no fixed width or sidebar border */
    .setup-tab {
        padding: 1.25rem 1rem;
        display: flex;
        flex-direction: column;
        gap: 0.25rem;
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

    .slot-value {
        font-size: 0.8rem;
        font-weight: 700;
        color: var(--color-accent-light);
        margin-left: 0.4rem;
        text-transform: none;
        letter-spacing: 0;
    }

    .slot-slider {
        width: 100%;
        cursor: pointer;
        accent-color: var(--color-accent);
    }

    .slot-slider:disabled {
        opacity: 0.35;
        cursor: default;
    }

    .slider-bounds {
        display: flex;
        justify-content: space-between;
        font-size: 0.7rem;
        color: var(--color-text-faint);
        margin-top: 0.1rem;
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

    .score-label {
        margin: 0.5rem 0 0.1rem;
        font-size: 0.7rem;
        color: var(--color-text-faint);
        line-height: 1.4;
    }

    .score-label:first-child {
        margin-top: 0;
    }

    .score-value {
        margin: 0;
        font-size: 0.85rem;
        font-weight: 600;
        color: var(--color-text);
    }

    .score-value.score-bad { color: var(--color-danger); }
    .score-value.score-warn { color: var(--color-warning); }

    .score-pending {
        font-style: italic;
        font-weight: 400;
        color: var(--color-text-faint);
    }

    .regen-btn {
        width: 100%;
        padding: 0.55rem 1rem;
        background: var(--color-surface);
        border: 1px solid var(--color-border);
        border-radius: var(--border-radius-sm);
        color: var(--color-text);
        font-size: 0.85rem;
        cursor: pointer;
        transition: background 0.15s, border-color 0.15s;
    }

    .regen-btn:hover:not(:disabled) {
        background: var(--color-accent-dim);
        border-color: var(--color-accent);
    }

    .regen-btn:disabled {
        opacity: 0.4;
        cursor: default;
    }
</style>
