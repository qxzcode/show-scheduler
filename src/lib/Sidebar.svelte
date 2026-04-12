<script lang="ts">
    import type { CustomConstraint } from "$lib/types";

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
        routineNames: string[];
        customConstraints: CustomConstraint[];
        constraintSatisfied: (boolean | null)[];
        onFile: (file: File) => void;
        onLoadDemo?: () => void;
        onDragOver: () => void;
        onDragLeave: () => void;
        onRegenerate?: () => void;
        onConstraintsChange: (c: CustomConstraint[]) => void;
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
        routineNames,
        customConstraints,
        constraintSatisfied,
        onFile,
        onLoadDemo,
        onDragOver,
        onDragLeave,
        onRegenerate,
        onConstraintsChange,
    }: Props = $props();

    function addConstraint() {
        const firstRoutine = routineNames[0] ?? '';
        onConstraintsChange([
            ...customConstraints,
            { id: `c${Date.now()}`, kind: 'in_slot', routine: firstRoutine, slot: 1 },
        ]);
    }

    function removeConstraint(id: string) {
        onConstraintsChange(customConstraints.filter(c => c.id !== id));
    }

    function updateConstraint(id: string, updates: Partial<CustomConstraint>) {
        onConstraintsChange(
            customConstraints.map(c => (c.id === id ? { ...c, ...updates } as CustomConstraint : c))
        );
    }

    function changeConstraintKind(id: string, newKind: 'in_slot' | 'directly_before') {
        const c = customConstraints.find(c => c.id === id);
        if (!c) return;
        let updated: CustomConstraint;
        if (newKind === 'in_slot') {
            // Preserve the "primary" routine (row 1): for directly_before that's beforeRoutine
            const routine = c.kind === 'directly_before' ? c.beforeRoutine : c.routine;
            updated = { id, kind: 'in_slot', routine, slot: 1 };
        } else {
            // Preserve the "primary" routine (row 1) as beforeRoutine
            const beforeRoutine = c.kind === 'in_slot' ? c.routine : c.beforeRoutine;
            const afterRoutine = routineNames.find(r => r !== beforeRoutine) ?? routineNames[0] ?? '';
            updated = { id, kind: 'directly_before', beforeRoutine, afterRoutine };
        }
        onConstraintsChange(customConstraints.map(c => (c.id === id ? updated : c)));
    }

    let aboutOpen = $state(false);

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
        {#if !fileName && onLoadDemo}
            <button class="demo-btn" onclick={onLoadDemo}>Try demo data</button>
        {/if}
        <ul class="csv-hints">
            <li><b>Header row:</b> routine names</li>
            <li><b>Cells below each header:</b> performers in that routine, one per cell</li>
        </ul>
    </section>

    {#if fileName}
        <section class="sidebar-section">
            <label class="field-label" for="num-slots">
                Show slots (incl. intermission)
                <span class="slot-value">{numSlots}</span>
            </label>
            <input
                class="slot-slider"
                type="range"
                id="num-slots"
                min={minSlots}
                max={maxSlots}
                step="1"
                bind:value={numSlots}
            />
            <div class="slider-bounds">
                <span>{minSlots}</span>
                <span>{maxSlots}</span>
            </div>
        </section>

        <section class="sidebar-section">
            <label class="field-label" for="intermission-tolerance">
                Intermission tolerance
                <span class="slot-value">{intermissionTolerance}</span>
                <span
                    class="info-icon"
                    title="How far off-center intermission is allowed to be (in slots). Lower = must stay near the middle of the show; higher = more flexibility, which can help reduce performer conflicts."
                    aria-label="About intermission tolerance"
                >ⓘ</span>
            </label>
            <input
                class="slot-slider"
                type="range"
                id="intermission-tolerance"
                min="0"
                max={maxIntermissionTolerance}
                step="1"
                bind:value={intermissionTolerance}
            />
            <div class="slider-bounds">
                <span>strict</span>
                <span>loose</span>
            </div>
        </section>
    {/if}

    {#if fileName}
        <section class="sidebar-section">
            <!-- svelte-ignore a11y_label_has_associated_control -->
            <label class="field-label">Custom Constraints</label>
            <div class="constraint-list">
                {#each customConstraints as constraint, i (constraint.id)}
                    <div
                        class="constraint-item"
                        class:constraint-violated={constraintSatisfied[i] === false}
                    >
                        <div class="constraint-row">
                            <span class="constraint-label">Put</span>
                            <select
                                class="constraint-select"
                                value={constraint.kind === 'in_slot' ? constraint.routine : constraint.beforeRoutine}
                                onchange={(e) => {
                                    const val = (e.target as HTMLSelectElement).value;
                                    if (constraint.kind === 'in_slot') {
                                        updateConstraint(constraint.id, { routine: val });
                                    } else {
                                        updateConstraint(constraint.id, { beforeRoutine: val });
                                    }
                                }}
                            >
                                {#each routineNames as name}
                                    <option value={name}>{name}</option>
                                {/each}
                            </select>
                            <button
                                class="constraint-delete"
                                onclick={() => removeConstraint(constraint.id)}
                                title="Remove constraint"
                            >✕</button>
                        </div>
                        <div class="constraint-row">
                            <select
                                class="constraint-type-select"
                                value={constraint.kind}
                                onchange={(e) => changeConstraintKind(constraint.id, (e.target as HTMLSelectElement).value as 'in_slot' | 'directly_before')}
                            >
                                <option value="directly_before">directly before</option>
                                <option value="in_slot">in slot</option>
                            </select>
                            {#if constraint.kind === 'in_slot'}
                                <input
                                    type="number"
                                    class="constraint-slot-input"
                                    min="1"
                                    max={numSlots}
                                    value={constraint.slot}
                                    onchange={(e) => {
                                        const raw = parseInt((e.target as HTMLInputElement).value);
                                        const clamped = Math.max(1, Math.min(numSlots, raw || 1));
                                        (e.target as HTMLInputElement).value = String(clamped);
                                        updateConstraint(constraint.id, { slot: clamped });
                                    }}
                                />
                            {:else}
                                <select
                                    class="constraint-select"
                                    value={constraint.afterRoutine}
                                    onchange={(e) => updateConstraint(constraint.id, { afterRoutine: (e.target as HTMLSelectElement).value })}
                                >
                                    {#each routineNames as name}
                                        <option value={name}>{name}</option>
                                    {/each}
                                </select>
                            {/if}
                        </div>
                    </div>
                {/each}
            </div>
            <button class="add-constraint-btn" onclick={addConstraint}>
                + New constraint
            </button>
        </section>
    {/if}

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

    <div class="sidebar-footer">
        <button class="footer-link" onclick={() => aboutOpen = true}>About</button>
        <span class="footer-sep">·</span>
        <a
            class="footer-github"
            href="https://github.com/qxzcode/show-scheduler"
            target="_blank"
            rel="noopener noreferrer"
            aria-label="GitHub repository"
        >
            <svg xmlns="http://www.w3.org/2000/svg" width="15" height="15" viewBox="0 0 98 98" fill="currentColor" aria-hidden="true">
                <path d="m41.4 69.4c-1.1.9-2.4 2.4-4 4-2.2 2.2-3.7 3.1-6.1 3.1s-4.4-1.3-6.3-4.1c-2.5-3.5-4.9-5.2-7.7-5.2-1.9 0-3.8.8-3.8 2 0 .6.4 1 1.5 1.1 2.2.3 3.4 1.9 4.4 4.4 2.7 6.5 6.4 9.9 12.8 9.9 1.5 0 3.2-.4 4.6-1v7.7c0 2.6-2.5 4.3-5.1 3.4C13.5 88 0 70.4 0 49.2 0 22.1 21.8 0 48.9 0S98 22.1 98 49.2c0 21.4-13.6 38.8-30.9 45.4-2.9 1.1-5.3-.5-5.3-3.4v-9.9c0-5.3-2.2-9.9-5.4-12C69.2 67.7 78.1 58.8 78.1 47c0-5-1.6-9.9-4.7-13.5 1.2-3.3 1.1-10-.3-12.5-3.8-.5-9.1 1.5-12 4.2-3.5-1.1-7.3-1.7-12-1.7-4.7 0-8.5.6-12.2 1.8-3-2.8-8.2-4.8-12-4.3-1.5 2.7-1.6 9.4-.4 12.6-2.9 3.4-4.6 8.6-4.6 13.4 0 11.8 8.9 20.9 21.5 22.4z" />
            </svg>
        </a>
    </div>
</div>

<svelte:window onkeydown={(e) => e.key === 'Escape' && aboutOpen && (aboutOpen = false)} />

{#if aboutOpen}
    <div
        class="about-backdrop"
        role="dialog"
        tabindex="-1"
        aria-modal="true"
        aria-label="About Show Scheduler"
        onclick={(e) => { if (e.target === e.currentTarget) aboutOpen = false; }}
        onkeydown={(e) => e.key === 'Escape' && (aboutOpen = false)}
    >
        <div class="about-dialog">
            <button class="about-close" onclick={() => aboutOpen = false} aria-label="Close">✕</button>
            <h2>Show Scheduler</h2>
            <p>
                This is a tool for building schedules for dance shows, created by Quinn&nbsp;Tucker for a club at RIT. Given a collection of routines to be performed, with some performers participating in multiple routines, it finds a routine order that:
            </p>
            <ul>
                <li>Minimizes the number of performers with back-to-back routines (no time to change costume).</li>
                <li>Keeps intermission near the middle of the show.</li>
                <li>Respects other custom constraints, such as putting a specific routine directly after another specific routine.</li>
            </ul>
            <p>
                Fun fact: I hate bloated websites! This entire app is a single ~100 KB static Svelte page. The core optimization algorithm is implemented in Rust and compiled to WebAssembly, running entirely in your browser. No data is uploaded to any server.
            </p>
            <p>
                View the source or report an issue on <a href="https://github.com/qxzcode/show-scheduler" target="_blank" rel="noopener noreferrer">GitHub</a>.
            </p>
        </div>
    </div>
{/if}

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
        flex: 1;
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

    .demo-btn {
        display: block;
        width: 100%;
        margin-top: 0.4rem;
        background: none;
        border: none;
        padding: 0;
        font-size: 0.75rem;
        color: var(--color-text-faint);
        cursor: pointer;
        text-align: center;
        text-decoration: underline;
        text-underline-offset: 0.15em;
    }

    .demo-btn:hover {
        color: var(--color-accent-light);
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

    .info-icon {
        font-size: 0.75rem;
        font-weight: 400;
        color: var(--color-text-faint);
        text-transform: none;
        letter-spacing: 0;
        margin-left: 0.3rem;
        cursor: help;
    }

    .slot-slider {
        width: 100%;
        cursor: pointer;
        accent-color: var(--color-accent);
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

    .regen-btn:hover {
        background: var(--color-accent-dim);
        border-color: var(--color-accent);
    }

    /* ── Custom constraints ──────────────────────────────────────────────────── */

    .constraint-list {
        display: flex;
        flex-direction: column;
        gap: 0.35rem;
        margin-bottom: 0.4rem;
    }

    .constraint-item {
        border: 1px solid var(--color-border);
        border-radius: var(--border-radius-sm);
        padding: 0.35rem 0.4rem;
        display: flex;
        flex-direction: column;
        gap: 0.2rem;
        background: var(--color-surface);
        transition: border-color 0.15s, background 0.15s;
    }

    .constraint-item.constraint-violated {
        border-color: var(--color-danger);
        background: var(--color-danger-dim);
    }

    .constraint-row {
        display: flex;
        align-items: center;
        gap: 0.25rem;
    }

    .constraint-label {
        font-size: 0.72rem;
        color: var(--color-text-muted);
        flex-shrink: 0;
    }

    .constraint-select,
    .constraint-type-select,
    .constraint-slot-input {
        background: var(--color-bg);
        border: 1px solid var(--color-border);
        border-radius: var(--border-radius-sm);
        color: var(--color-text);
        font-size: 0.75rem;
        padding: 0.15rem 0.25rem;
        cursor: pointer;
    }

    .constraint-select {
        flex: 1;
        min-width: 0;
    }

    .constraint-type-select {
        flex-shrink: 0;
    }

    .constraint-slot-input {
        width: 3.2rem;
        flex-shrink: 0;
        cursor: text;
    }

    .constraint-delete {
        flex-shrink: 0;
        background: none;
        border: none;
        color: var(--color-text-faint);
        cursor: pointer;
        padding: 0.1rem 0.2rem;
        font-size: 0.75rem;
        line-height: 1;
        margin-left: auto;
    }

    .constraint-delete:hover {
        color: var(--color-danger);
    }

    .add-constraint-btn {
        width: 100%;
        padding: 0.35rem;
        background: none;
        border: 1px dashed var(--color-border);
        border-radius: var(--border-radius-sm);
        color: var(--color-text-faint);
        font-size: 0.78rem;
        cursor: pointer;
        transition: border-color 0.15s, color 0.15s;
    }

    .add-constraint-btn:hover {
        border-color: var(--color-accent);
        color: var(--color-accent-light);
    }

    /* ── Sidebar footer ──────────────────────────────────────────────────────── */

    .sidebar-footer {
        margin-top: auto;
        padding-top: 1rem;
        display: flex;
        align-items: center;
        justify-content: center;
        gap: 0.4rem;
        font-size: 0.75rem;
        color: var(--color-text-faint);
    }

    .footer-link {
        background: none;
        border: none;
        padding: 0;
        font-size: 0.75rem;
        color: var(--color-text-faint);
        cursor: pointer;
    }

    .footer-link:hover {
        color: var(--color-accent-light);
    }

    .footer-sep {
        user-select: none;
    }

    .footer-github {
        display: flex;
        align-items: center;
        color: var(--color-text-faint);
        line-height: 1;
    }

    .footer-github:hover {
        color: var(--color-accent-light);
    }

    /* ── About dialog ────────────────────────────────────────────────────────── */

    .about-backdrop {
        position: fixed;
        inset: 0;
        background: rgba(0, 0, 0, 0.55);
        display: flex;
        align-items: center;
        justify-content: center;
        z-index: 100;
    }

    .about-dialog {
        position: relative;
        background: var(--color-panel);
        border: 1px solid var(--color-border);
        border-radius: var(--border-radius-sm);
        padding: 1.5rem 1.75rem;
        max-width: 30rem;
        width: calc(100% - 2rem);
        max-height: 90vh;
        overflow-y: auto;
        box-shadow: 0 8px 32px rgba(0, 0, 0, 0.4);
    }

    .about-close {
        position: absolute;
        top: 0.6rem;
        right: 0.75rem;
        background: none;
        border: none;
        color: var(--color-text-faint);
        font-size: 0.85rem;
        cursor: pointer;
        padding: 0.2rem 0.3rem;
        line-height: 1;
    }

    .about-close:hover {
        color: var(--color-text);
    }

    .about-dialog h2 {
        margin: 0 0 0.75rem;
        font-size: 1rem;
        font-weight: 700;
        color: var(--color-accent-light);
        letter-spacing: 0.02em;
    }

    .about-dialog * {
        font-size: 0.85rem;
        color: var(--color-text-muted);
        line-height: 1.6;
    }

    .about-dialog *:last-child {
        margin-bottom: 0;
    }

    .about-dialog * a {
        color: var(--color-accent-light);
        text-decoration: underline;
        text-underline-offset: 0.15em;
    }

    .about-dialog * a:hover {
        color: var(--color-text);
    }
</style>
