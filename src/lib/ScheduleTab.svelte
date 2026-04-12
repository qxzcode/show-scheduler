<script lang="ts">
    import NoFile from "./NoFile.svelte";

    interface ConflictGroup {
        routine: string;
        dancers: string[];
    }

    interface SlotResult {
        slot_number: number;
        routines: string[];
        dist1_conflicts: ConflictGroup[];
        dist2_conflicts: ConflictGroup[];
    }

    interface Props {
        slots: SlotResult[];
        hasStarted: boolean; // optimization has been triggered but no result yet
        error: string;
    }

    let { slots, hasStarted, error }: Props = $props();

    let copied = $state(false);

    function copySchedule() {
        const tsv = slots.map(s => `${s.slot_number}\t${s.routines.join(" + ")}`).join("\n");
        navigator.clipboard.writeText(tsv).then(() => {
            copied = true;
            setTimeout(() => copied = false, 2000);
        });
    }

    function formatNames(names: string[]): string {
        if (names.length === 1) return names[0];
        if (names.length === 2) return `${names[0]} and ${names[1]}`;
        return `${names.slice(0, -1).join(", ")}, and ${names[names.length - 1]}`;
    }

    function isAre(names: string[]): string {
        return names.length === 1 ? "is" : "are";
    }
</script>

<div class="schedule-tab">
    {#if error}
        <div class="empty-state error-state">
            <p class="error-label">Error</p>
            <p class="error-message">{error}</p>
        </div>
    {:else if slots.length === 0}
        {#if hasStarted}
            <div class="empty-state">
                <p>Optimizing…</p>
            </div>
        {:else}
            <NoFile />
        {/if}
    {:else}
        <button class="copy-btn" class:copy-btn--copied={copied} onclick={copySchedule} title="Copy schedule">
            {#if copied}
                Copied!
            {:else}
                <svg xmlns="http://www.w3.org/2000/svg" width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><rect width="14" height="14" x="8" y="8" rx="2" ry="2"/><path d="M4 16c-1.1 0-2-.9-2-2V4c0-1.1.9-2 2-2h10c1.1 0 2 .9 2 2"/></svg>
                Copy schedule
            {/if}
        </button>
        <table class="schedule-table">
            <thead>
                <tr>
                    <th class="col-slot">#</th>
                    <th class="col-routines">Routine(s)</th>
                    <th class="col-conflicts">Conflicts</th>
                </tr>
            </thead>
            <tbody>
                {#each slots as slot}
                    {@const isIntermission = slot.routines.length === 1 && slot.routines[0] === "[Intermission]"}
                    {@const hasDist1 = slot.dist1_conflicts.length > 0}
                    {@const hasDist2 = slot.dist2_conflicts.length > 0}
                    <tr class={[isIntermission && "row-intermission", hasDist1 && "row-dist1", !hasDist1 && hasDist2 && "row-dist2"].filter(Boolean).join(" ")}>
                        <td class="col-slot">{slot.slot_number}</td>
                        <td class="col-routines">{slot.routines.join(" + ")}</td>
                        <td class="col-conflicts">
                            {#each slot.dist1_conflicts as group}
                                <span class="conflict-names dist1"><strong>{formatNames(group.dancers)}</strong> {isAre(group.dancers)} also in <strong>{group.routine}</strong>.</span>
                            {/each}
                            {#each slot.dist2_conflicts as group}
                                <span class="conflict-names dist2"><strong>{formatNames(group.dancers)}</strong> {isAre(group.dancers)} also in <strong>{group.routine}</strong>.</span>
                            {/each}
                        </td>
                    </tr>
                {/each}
            </tbody>
        </table>
    {/if}
</div>

<style>
    .schedule-tab {
        padding: 1.5rem;
    }

    .copy-btn {
        position: fixed;
        bottom: 1.5rem;
        right: 1.5rem;
        z-index: 10;
        display: flex;
        align-items: center;
        gap: 0.4rem;
        padding: 0.35rem 0.75rem;
        background: var(--color-panel);
        border: 1px solid var(--color-border);
        border-radius: var(--border-radius-sm);
        color: var(--color-text-muted);
        font-size: 0.78rem;
        font-weight: 500;
        cursor: pointer;
        transition: border-color 0.15s, color 0.15s;
    }

    .copy-btn:hover {
        border-color: var(--color-accent);
        color: var(--color-accent-light);
    }

    .copy-btn--copied {
        border-color: var(--color-success);
        color: var(--color-success);
    }

    .empty-state {
        display: flex;
        align-items: center;
        justify-content: center;
        min-height: 12rem;
        color: var(--color-text-faint);
        font-size: 1rem;
        font-style: italic;
    }

    .error-state {
        flex-direction: column;
        gap: 0.5rem;
        font-style: normal;
    }

    .error-label {
        margin: 0;
        font-weight: 700;
        color: var(--color-danger);
        font-size: 0.85rem;
        text-transform: uppercase;
        letter-spacing: 0.06em;
    }

    .error-message {
        margin: 0;
        color: var(--color-text-muted);
        font-size: 0.9rem;
        font-family: monospace;
    }

    .schedule-table {
        width: 100%;
        border-collapse: collapse;
        font-size: 0.875rem;
    }

    .schedule-table th {
        text-align: left;
        font-size: 0.72rem;
        font-weight: 600;
        text-transform: uppercase;
        letter-spacing: 0.06em;
        color: var(--color-text-faint);
        padding: 0.5rem 0.75rem;
        border-bottom: 1px solid var(--color-border);
        background: var(--color-panel);
        position: sticky;
        top: var(--sticky-top, 0);
    }

    .schedule-table td {
        padding: 0.4rem 0.75rem;
        border-bottom: 1px solid var(--color-border);
        vertical-align: middle;
        color: var(--color-text);
    }

    .schedule-table tr:last-child td {
        border-bottom: none;
    }

    .col-slot {
        width: 2.5rem;
        color: var(--color-text-faint);
        font-variant-numeric: tabular-nums;
    }

    .col-routines {
        width: 1%;
        white-space: nowrap;
        font-weight: 500;
        padding-right: 1.5rem;
    }

    .col-conflicts {
        /* fills remaining width */
    }

    .col-conflicts .conflict-names + .conflict-names {
        display: block;
        margin-top: 0.2rem;
    }

    .row-intermission td {
        background: var(--color-intermission) !important;
        font-style: italic;
        color: var(--color-text-muted);
    }

    .row-dist1 td {
        background: var(--color-danger-dim);
    }

    .row-dist1 td:first-child {
        border-left: 2px solid var(--color-danger);
    }

    .row-dist2 td {
        background: var(--color-warning-dim);
    }

    .row-dist2 td:first-child {
        border-left: 2px solid var(--color-warning);
    }

    .conflict-names {
        font-size: 0.78rem;
    }

    .conflict-names.dist1 { color: var(--color-danger); }
    .conflict-names.dist2 { color: var(--color-warning); }
</style>
