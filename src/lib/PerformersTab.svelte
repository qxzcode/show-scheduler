<script lang="ts">
    import { suggestMerges, type SuggestedMerge } from "./similarity.js";

    interface Props {
        /** Map of dancer name → set of routine names they appear in */
        performerRoutines: Map<string, Set<string>>;
        /** Alias map: alias → canonical name. Canonical names are not in this map as keys. */
        aliasMap: Map<string, string>;
        dismissedSuggestions: Set<string>;
        onAliasMapChange: (map: Map<string, string>) => void;
        onDismissedChange: (set: Set<string>) => void;
        onGoToSchedule: () => void;
    }

    let { performerRoutines, aliasMap, dismissedSuggestions, onAliasMapChange, onDismissedChange, onGoToSchedule }: Props = $props();

    // Build routine-count map for similarity computation (count only non-aliased or canonical)
    let routineCounts = $derived(
        new Map([...performerRoutines.entries()].map(([name, routines]) => [name, routines.size]))
    );

    let suggestions = $derived(suggestMerges(routineCounts, performerRoutines));

    let visibleSuggestions = $derived(
        suggestions.filter(s => !dismissedSuggestions.has(suggestionKey(s)))
    );

    function suggestionKey(s: SuggestedMerge) {
        return [...s.names].sort().join("\0");
    }

    function dismiss(s: SuggestedMerge) {
        onDismissedChange(new Set([...dismissedSuggestions, suggestionKey(s)]));
    }

    function acceptSuggestion(s: SuggestedMerge) {
        const newMap = new Map(aliasMap);
        for (const name of s.names) {
            if (name !== s.canonical) newMap.set(name, s.canonical);
            else newMap.delete(name); // ensure canonical isn't aliased to itself
        }
        onAliasMapChange(newMap);
        onDismissedChange(new Set([...dismissedSuggestions, suggestionKey(s)]));
    }

    // Per-suggestion canonical override
    let suggestionCanonicals = $state(new Map<string, string>());

    function getCanonical(s: SuggestedMerge): string {
        return suggestionCanonicals.get(suggestionKey(s)) ?? s.canonical;
    }

    function setCanonical(s: SuggestedMerge, name: string) {
        const key = suggestionKey(s);
        suggestionCanonicals = new Map([...suggestionCanonicals, [key, name]]);
    }

    // Manual merge: floating menu state
    let mergeMenuTarget = $state<string | null>(null);
    interface MenuPos {
        right: number;
        top?: number;
        bottom?: number;
        maxHeight: number;
    }
    let mergeMenuPos = $state<MenuPos | null>(null);

    function openMergeMenu(name: string, btn: HTMLButtonElement) {
        mergeMenuTarget = name;
        const r = btn.getBoundingClientRect();
        const margin = 8;
        const spaceBelow = window.innerHeight - r.bottom - margin;
        const spaceAbove = r.top - margin;
        const right = window.innerWidth - r.right;

        if (spaceBelow >= spaceAbove || spaceBelow >= 200) {
            mergeMenuPos = { right, top: r.bottom + 4, maxHeight: spaceBelow };
        } else {
            mergeMenuPos = { right, bottom: window.innerHeight - r.top + 4, maxHeight: spaceAbove };
        }
    }

    function closeMergeMenu() {
        mergeMenuTarget = null;
        mergeMenuPos = null;
    }

    // Resolve canonical for display
    function resolveCanonical(name: string): string {
        return aliasMap.get(name) ?? name;
    }

    function isAlias(name: string): boolean {
        return aliasMap.has(name);
    }

    function unmerge(alias: string) {
        const newMap = new Map(aliasMap);
        newMap.delete(alias);
        onAliasMapChange(newMap);
    }

    // All canonical names (names not aliased to something else)
    let canonicals = $derived(
        [...performerRoutines.keys()].filter(n => !aliasMap.has(n)).sort((a, b) => a.localeCompare(b, undefined, { sensitivity: "base" }))
    );

    // All alias names
    let aliases = $derived(
        [...aliasMap.entries()].sort(([a], [b]) => a.localeCompare(b))
    );

    function mergeInto(alias: string, canonical: string) {
        if (alias === canonical) return;
        const newMap = new Map(aliasMap);
        newMap.set(alias, canonical);
        onAliasMapChange(newMap);
        closeMergeMenu();
    }
</script>

<svelte:window onkeydown={(e) => e.key === "Escape" && closeMergeMenu()} />

<div class="performers-tab">

    {#if visibleSuggestions.length > 0}
        <section class="suggestions-section">
            <h2 class="section-title">
                <span class="badge">{visibleSuggestions.length}</span>
                Suggested duplicates
            </h2>
            <p class="section-desc">These names look like they may refer to the same person. Merge them under one canonical name?</p>
            <div class="suggestions-list">
                {#each visibleSuggestions as s (suggestionKey(s))}
                    {@const key = suggestionKey(s)}
                    <div class="suggestion-card">
                        <div class="suggestion-names">
                            {#each s.names as name}
                                <button
                                    class={["name-chip", getCanonical(s) === name && "is-canonical"].filter(Boolean).join(" ")}
                                    onclick={() => setCanonical(s, name)}
                                    title="Set as canonical name"
                                >
                                    {name}
                                    <span class="routine-count">{routineCounts.get(name) ?? 0} routine{(routineCounts.get(name) ?? 0) === 1 ? "" : "s"}</span>
                                </button>
                            {/each}
                        </div>
                        <p class="canonical-hint">
                            Canonical: <strong>{getCanonical(s)}</strong>
                            <span class="hint-sub">(click a name above to change)</span>
                        </p>
                        <div class="suggestion-actions">
                            <button class="btn btn-accept" onclick={() => { const canon = getCanonical(s); acceptSuggestion({ ...s, canonical: canon }); }}>
                                Merge
                            </button>
                            <button class="btn btn-dismiss" onclick={() => dismiss(s)}>
                                Dismiss
                            </button>
                        </div>
                    </div>
                {/each}
            </div>
        </section>
    {/if}

    {#if visibleSuggestions.length === 0}
        <div class="schedule-nudge">
            Once you've verified that all the performer names below refer to unique people,
            <button class="nudge-btn" onclick={onGoToSchedule}>go to the optimized schedule →</button>
        </div>
    {/if}

    <section class="performers-section">
        <h2 class="section-title">All performers</h2>

        {#if aliases.length > 0}
            <div class="alias-list">
                <p class="alias-list-label">Currently merged:</p>
                {#each aliases as [alias, canonical]}
                    <div class="alias-row">
                        <span class="alias-name">{alias}</span>
                        <span class="alias-arrow">→</span>
                        <span class="canonical-name">{canonical}</span>
                        <button class="btn-unmerge" onclick={() => unmerge(alias)} title="Unmerge">✕</button>
                    </div>
                {/each}
            </div>
        {/if}

        <div class="table-scroll">
            <table class="performers-table">
                <thead>
                    <tr>
                        <th>Performer</th>
                        <th>Routines</th>
                        <th></th>
                    </tr>
                </thead>
                <tbody>
                    {#each canonicals as name}
                        <tr>
                            <td class="performer-name">{name}</td>
                            <td class="routine-tags">
                                {#each [...(performerRoutines.get(name) ?? [])] as routine}
                                    <span class="routine-tag">{routine}</span>
                                {/each}
                            </td>
                            <td class="merge-cell">
                                <button class="btn-merge-into" onclick={(e) => openMergeMenu(name, e.currentTarget)}>Merge with…</button>
                            </td>
                        </tr>
                    {/each}
                </tbody>
            </table>
        </div>
    </section>
</div>

{#if mergeMenuTarget && mergeMenuPos}
    <button class="merge-backdrop" onclick={closeMergeMenu} aria-label="Close menu" tabindex="-1"></button>
    <div class="merge-menu" style="right: {mergeMenuPos.right}px; max-height: {mergeMenuPos.maxHeight}px; {mergeMenuPos.top != null ? `top: ${mergeMenuPos.top}px` : `bottom: ${mergeMenuPos.bottom}px`}">
        {#each canonicals.filter(c => c !== mergeMenuTarget) as c}
            <button class="merge-option" onclick={() => mergeInto(mergeMenuTarget!, c)}>{c}</button>
        {/each}
    </div>
{/if}

<style>
    .performers-tab {
        padding: 1.5rem;
    }

    .section-title {
        font-size: 1rem;
        font-weight: 600;
        color: var(--color-text);
        margin: 0 0 0.4rem 0;
        display: flex;
        align-items: center;
        gap: 0.5rem;
    }

    .badge {
        display: inline-flex;
        align-items: center;
        justify-content: center;
        background: var(--color-accent);
        color: #fff;
        font-size: 0.7rem;
        font-weight: 700;
        border-radius: 999px;
        min-width: 1.4em;
        height: 1.4em;
        padding: 0 0.3em;
    }

    .section-desc {
        font-size: 0.85rem;
        color: var(--color-text-muted);
        margin: 0 0 0.75rem 0;
    }

    .suggestions-section {
        margin-bottom: 2rem;
    }

    .suggestions-list {
        display: flex;
        flex-direction: column;
        gap: 0.75rem;
    }

    .suggestion-card {
        background: var(--color-surface);
        border: 1px solid var(--color-accent);
        border-radius: var(--border-radius);
        padding: 0.875rem 1rem;
    }

    .suggestion-names {
        display: flex;
        flex-wrap: wrap;
        gap: 0.5rem;
        margin-bottom: 0.5rem;
    }

    .name-chip {
        display: flex;
        align-items: center;
        gap: 0.4rem;
        padding: 0.3rem 0.65rem;
        border-radius: 999px;
        border: 1px solid var(--color-border);
        background: var(--color-surface-alt);
        color: var(--color-text);
        font-size: 0.85rem;
        cursor: pointer;
        transition: border-color 0.1s, background 0.1s;
    }

    .name-chip:hover {
        border-color: var(--color-accent-light);
    }

    .name-chip.is-canonical {
        border-color: var(--color-accent);
        background: var(--color-accent-dim);
        color: var(--color-accent-light);
        font-weight: 600;
    }

    .routine-count {
        font-size: 0.7rem;
        color: var(--color-text-faint);
    }

    .name-chip.is-canonical .routine-count {
        color: var(--color-accent-light);
        opacity: 0.7;
    }

    .canonical-hint {
        font-size: 0.8rem;
        color: var(--color-text-muted);
        margin: 0 0 0.6rem 0;
    }

    .hint-sub {
        font-size: 0.75rem;
        color: var(--color-text-faint);
    }

    .suggestion-actions {
        display: flex;
        gap: 0.5rem;
    }

    .btn {
        padding: 0.3rem 0.9rem;
        border-radius: var(--border-radius-sm);
        border: none;
        font-size: 0.8rem;
        font-weight: 600;
        cursor: pointer;
        transition: opacity 0.1s;
    }

    .btn:hover { opacity: 0.85; }

    .btn-accept {
        background: var(--color-accent);
        color: #fff;
    }

    .btn-dismiss {
        background: var(--color-surface-alt);
        color: var(--color-text-muted);
        border: 1px solid var(--color-border);
    }

    .schedule-nudge {
        font-size: 0.875rem;
        color: var(--color-text-muted);
        margin-bottom: 1.5rem;
        line-height: 1.6;
    }

    .nudge-btn {
        background: none;
        border: none;
        padding: 0;
        color: var(--color-accent-light);
        font-size: inherit;
        font-weight: 600;
        cursor: pointer;
        text-decoration: underline;
        text-underline-offset: 0.2em;
    }

    .nudge-btn:hover {
        color: var(--color-accent);
    }

    .performers-section {
        /* no extra margin needed */
    }

    .alias-list {
        background: var(--color-surface);
        border: 1px solid var(--color-border);
        border-radius: var(--border-radius-sm);
        padding: 0.6rem 0.75rem;
        margin-bottom: 1rem;
    }

    .alias-list-label {
        font-size: 0.75rem;
        font-weight: 600;
        text-transform: uppercase;
        letter-spacing: 0.05em;
        color: var(--color-text-faint);
        margin: 0 0 0.4rem 0;
    }

    .alias-row {
        display: flex;
        align-items: center;
        gap: 0.5rem;
        font-size: 0.85rem;
        padding: 0.15rem 0;
    }

    .alias-name {
        color: var(--color-text-muted);
        font-style: italic;
    }

    .alias-arrow {
        color: var(--color-text-faint);
    }

    .canonical-name {
        color: var(--color-accent-light);
        font-weight: 500;
    }

    .btn-unmerge {
        background: none;
        border: none;
        color: var(--color-text-faint);
        cursor: pointer;
        font-size: 0.75rem;
        padding: 0 0.25rem;
        line-height: 1;
        margin-left: auto;
    }

    .btn-unmerge:hover { color: var(--color-danger); }

    .table-scroll {
        overflow-x: auto;
        -webkit-overflow-scrolling: touch;
    }

    .performers-table {
        width: 100%;
        border-collapse: collapse;
        font-size: 0.875rem;
    }

    .performers-table th {
        text-align: left;
        font-size: 0.72rem;
        font-weight: 600;
        text-transform: uppercase;
        letter-spacing: 0.06em;
        color: var(--color-text-faint);
        padding: 0.4rem 0.6rem;
        border-bottom: 1px solid var(--color-border);
    }

    .performers-table td {
        padding: 0.35rem 0.6rem;
        border-bottom: 1px solid var(--color-border);
        vertical-align: middle;
    }

    .performers-table tr:last-child td {
        border-bottom: none;
    }

    .performers-table tbody tr:hover td {
        background: color-mix(in srgb, var(--color-surface) 50%, transparent);
    }

    .performer-name {
        color: var(--color-text);
        white-space: nowrap;
        font-weight: 500;
    }

    .routine-tags {
        display: flex;
        flex-wrap: wrap;
        gap: 0.3rem;
    }

    .routine-tag {
        font-size: 0.72rem;
        padding: 0.1rem 0.45rem;
        background: var(--color-surface-alt);
        border: 1px solid var(--color-border);
        border-radius: 999px;
        color: var(--color-text-muted);
        white-space: nowrap;
    }

    .merge-cell {
        white-space: nowrap;
        text-align: right;
    }

    .btn-merge-into {
        background: none;
        border: 1px solid var(--color-border);
        border-radius: var(--border-radius-sm);
        color: var(--color-text-faint);
        font-size: 0.75rem;
        padding: 0.2rem 0.5rem;
        cursor: pointer;
    }

    .btn-merge-into:hover {
        border-color: var(--color-accent);
        color: var(--color-accent-light);
    }

    .merge-backdrop {
        position: fixed;
        inset: 0;
        z-index: 99;
        background: none;
        border: none;
        cursor: default;
        padding: 0;
    }

    .merge-menu {
        position: fixed;
        z-index: 100;
        background: var(--color-surface);
        border: 1px solid var(--color-border);
        border-radius: var(--border-radius);
        box-shadow: 0 0.5rem 1.5rem #0006;
        min-width: 12rem;
        overflow-y: auto;
        display: flex;
        flex-direction: column;
        padding: 0.25rem;
    }

    .merge-option {
        background: none;
        border: none;
        border-radius: var(--border-radius-sm);
        color: var(--color-text);
        font-size: 0.85rem;
        padding: 0.4rem 0.6rem;
        text-align: left;
        cursor: pointer;
        white-space: nowrap;
    }

    .merge-option:hover {
        background: var(--color-accent-dim);
        color: var(--color-accent-light);
    }
</style>
