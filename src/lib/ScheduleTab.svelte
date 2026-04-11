<script lang="ts">
    interface SlotResult {
        slot_number: number;
        routines: string[];
        dist1_conflicts: string[];
        dist2_conflicts: string[];
    }

    interface Props {
        slots: SlotResult[];
        hasStarted: boolean; // optimization has been triggered but no result yet
        error: string;
    }

    let { slots, hasStarted, error }: Props = $props();
</script>

<div class="schedule-tab">
    {#if error}
        <div class="empty-state error-state">
            <p class="error-label">Error</p>
            <p class="error-message">{error}</p>
        </div>
    {:else if slots.length === 0}
        <div class="empty-state">
            {#if hasStarted}
                <p>Optimizing…</p>
            {:else}
                <p>Select a CSV to begin.</p>
            {/if}
        </div>
    {:else}
        <table class="schedule-table">
            <thead>
                <tr>
                    <th class="col-slot">#</th>
                    <th class="col-routines">Routine(s)</th>
                    <th class="col-conflicts">Distance-1 conflicts</th>
                    <th class="col-conflicts">Distance-2 conflicts</th>
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
                            {#if hasDist1}
                                <span class="conflict-names dist1">{slot.dist1_conflicts.join(", ")}</span>
                            {/if}
                        </td>
                        <td class="col-conflicts">
                            {#if hasDist2}
                                <span class="conflict-names dist2">{slot.dist2_conflicts.join(", ")}</span>
                            {/if}
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
        top: 0;
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

    .schedule-table tr:hover td {
        background: var(--color-surface);
    }

    .col-slot {
        width: 2.5rem;
        color: var(--color-text-faint);
        font-variant-numeric: tabular-nums;
    }

    .col-routines {
        font-weight: 500;
    }

    .col-conflicts {
        width: 30%;
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
