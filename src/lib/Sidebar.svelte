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
</style>
