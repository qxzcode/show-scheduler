/** Jaro-Winkler similarity, returns 0..1 (1 = identical). */
export function jaroWinkler(a: string, b: string): number {
    if (a === b) return 1;
    const la = a.length, lb = b.length;
    if (la === 0 || lb === 0) return 0;

    const matchDist = Math.max(Math.floor(Math.max(la, lb) / 2) - 1, 0);
    const aMatched = new Uint8Array(la);
    const bMatched = new Uint8Array(lb);
    let matches = 0, transpositions = 0;

    for (let i = 0; i < la; i++) {
        const lo = Math.max(0, i - matchDist);
        const hi = Math.min(i + matchDist + 1, lb);
        for (let j = lo; j < hi; j++) {
            if (bMatched[j] || a[i] !== b[j]) continue;
            aMatched[i] = bMatched[j] = 1;
            matches++;
            break;
        }
    }
    if (matches === 0) return 0;

    let k = 0;
    for (let i = 0; i < la; i++) {
        if (!aMatched[i]) continue;
        while (!bMatched[k]) k++;
        if (a[i] !== b[k]) transpositions++;
        k++;
    }

    const jaro = (matches / la + matches / lb + (matches - transpositions / 2) / matches) / 3;

    // Winkler prefix bonus (up to 4 chars)
    let prefix = 0;
    for (let i = 0; i < Math.min(4, la, lb); i++) {
        if (a[i] !== b[i]) break;
        prefix++;
    }
    return jaro + prefix * 0.1 * (1 - jaro);
}

export interface SuggestedMerge {
    names: string[];      // all names in this group, most-used first
    canonical: string;    // auto-chosen canonical (most routines, then longest)
}

/**
 * Given a map of name → routine-count, find pairs whose Jaro-Winkler
 * similarity exceeds `threshold` and return them as merge suggestions.
 * Names that differ only in case are always grouped.
 * Pairs that appear in the same routine are never suggested (they must be different people).
 */
export function suggestMerges(
    routineCounts: Map<string, number>,
    performerRoutines: Map<string, Set<string>>,
    threshold = 0.88,
): SuggestedMerge[] {
    const names = [...routineCounts.keys()];
    const n = names.length;

    // Union-find
    const parent = names.map((_, i) => i);
    function find(i: number): number {
        while (parent[i] !== i) { parent[i] = parent[parent[i]]; i = parent[i]; }
        return i;
    }
    function union(i: number, j: number) { parent[find(i)] = find(j); }

    for (let i = 0; i < n; i++) {
        for (let j = i + 1; j < n; j++) {
            const ai = names[i].toLowerCase(), bj = names[j].toLowerCase();
            if (ai === bj || jaroWinkler(ai, bj) >= threshold) {
                // If they share a routine they are definitely different people — skip.
                const ri = performerRoutines.get(names[i]);
                const rj = performerRoutines.get(names[j]);
                if (ri && rj && [...ri].some(r => rj.has(r))) continue;
                union(i, j);
            }
        }
    }

    // Group by root
    const groups = new Map<number, string[]>();
    for (let i = 0; i < n; i++) {
        const root = find(i);
        if (!groups.has(root)) groups.set(root, []);
        groups.get(root)!.push(names[i]);
    }

    return [...groups.values()]
        .filter(g => g.length > 1)
        .map(g => {
            g.sort((a, b) => {
                const dc = (routineCounts.get(b) ?? 0) - (routineCounts.get(a) ?? 0);
                if (dc !== 0) return dc;
                return b.length - a.length;
            });
            return { names: g, canonical: g[0] };
        });
}
