<script lang="ts">
    import OptimizerWorker from "$lib/optimizer.worker.ts?worker";
    import Sidebar from "$lib/Sidebar.svelte";
    import PerformersTab from "$lib/PerformersTab.svelte";
    import ScheduleTab from "$lib/ScheduleTab.svelte";
    import NoFile from "$lib/NoFile.svelte";
    import type { CustomConstraint } from "$lib/types";

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

    interface OptimizeResult {
        slots: SlotResult[];
        score: [number, number, number];
    }

    // ── File / input state ────────────────────────────────────────────────────
    let csvText = $state("");
    let fileName = $state("");
    let numSlots = $state(32);
    let intermissionTolerance = $state(2);
    let dragOver = $state(false);

    // ── Alias map: alias → canonical ─────────────────────────────────────────
    let aliasMap = $state(new Map<string, string>());
    let dismissedSuggestions = $state(new Set<string>());

    // ── Custom constraints ────────────────────────────────────────────────────
    let customConstraints = $state<CustomConstraint[]>([]);

    // ── Optimizer state ───────────────────────────────────────────────────────
    let slots = $state<SlotResult[]>([]);
    let score = $state<{ d1: number; d2: number; mid: number } | null>(null);
    let status = $state("");
    let error = $state("");
    let running = $state(false);
    let hasStarted = $state(false); // triggered at least once
    let worker: Worker | null = null;

    // ── Responsive layout ─────────────────────────────────────────────────────
    let isMobile = $state(false);

    $effect(() => {
        const mq = window.matchMedia("(max-width: 40rem)");
        isMobile = mq.matches;
        if (mq.matches) activeTab = "setup";
        const handler = (e: MediaQueryListEvent) => {
            isMobile = e.matches;
            if (e.matches && activeTab !== "performers" && activeTab !== "schedule") {
                activeTab = "setup";
            }
            if (!e.matches && activeTab === "setup") activeTab = "performers";
        };
        mq.addEventListener("change", handler);
        return () => mq.removeEventListener("change", handler);
    });

    // ── Tab state ─────────────────────────────────────────────────────────────
    type Tab = "setup" | "performers" | "schedule";
    let activeTab = $state<Tab>("performers");
    let scheduleHasNewResult = $state(false);

    // ── Name normalization ────────────────────────────────────────────────────
    /**
     * Canonical form for a name: trim, collapse whitespace, and normalize
     * punctuation variants (curly quotes → straight, fancy dashes → hyphen).
     */
    function normalizeName(s: string): string {
        return s
            .trim()
            .replace(/\s+/g, " ")
            .replace(/[\u2018\u2019\u201A\u201B\u2032\u2035`]/g, "'") // single-quote variants
            .replace(/[\u201C\u201D\u201E\u201F\u2033\u2036]/g, '"')   // double-quote variants
            .replace(/[\u2010\u2011\u2012\u2013\u2014\u2015]/g, "-");  // dash variants
    }

    // ── CSV field parsing (RFC 4180 quoted fields) ────────────────────────────
    function parseCSVLine(line: string): string[] {
        const fields: string[] = [];
        let i = 0;
        while (i < line.length) {
            if (line[i] === '"') {
                // Quoted field: read until closing unescaped quote
                let field = '';
                i++; // skip opening quote
                while (i < line.length) {
                    if (line[i] === '"') {
                        if (line[i + 1] === '"') { field += '"'; i += 2; } // escaped quote
                        else { i++; break; }                               // closing quote
                    } else {
                        field += line[i++];
                    }
                }
                fields.push(field);
                if (line[i] === ',') i++; // skip delimiter
            } else {
                // Unquoted field
                const end = line.indexOf(',', i);
                if (end === -1) { fields.push(line.slice(i)); break; }
                fields.push(line.slice(i, end));
                i = end + 1;
            }
        }
        if (line.endsWith(',')) fields.push(''); // trailing empty field
        return fields;
    }

    // ── Derived: performer → routines map (normalized via aliasMap) ───────────
    let performerRoutines = $derived(buildPerformerRoutines(csvText, aliasMap));

    function buildPerformerRoutines(csv: string, aliases: Map<string, string>): Map<string, Set<string>> {
        const map = new Map<string, Set<string>>();
        if (!csv.trim()) return map;
        const lines = csv.trim().split(/\r?\n/);
        const header = lines[0] ? parseCSVLine(lines[0]).map(normalizeName) : [];
        for (let col = 0; col < header.length; col++) {
            const routineName = header[col];
            if (!routineName || routineName === "[Intermission]") continue;
            for (let row = 1; row < lines.length; row++) {
                const cells = parseCSVLine(lines[row]);
                const raw = normalizeName(cells[col] ?? "");
                if (!raw) continue;
                const canonical = aliases.get(raw) ?? raw;
                if (!map.has(canonical)) map.set(canonical, new Set());
                map.get(canonical)!.add(routineName);
            }
        }
        return map;
    }

    // ── Normalized CSV: normalize names, then replace aliases with canonicals ──
    let normalizedCsv = $derived(normalizeCsv(csvText, aliasMap));

    // ── Slot range: min/max valid show slots based on doubleable routines ─────
    let slotRange = $derived(computeSlotRange(normalizedCsv));

    function computeSlotRange(csv: string): { min: number; max: number } {
        if (!csv.trim()) return { min: 1, max: 100 };
        const lines = csv.trim().split(/\r?\n/);
        const header = lines[0] ? parseCSVLine(lines[0]).map(s => s.trim()) : [];
        // +1 for the [Intermission] automatically appended by parse_csv
        const totalRoutines = header.filter(Boolean).length + 1;
        const dataRows = lines.slice(1).map(l => parseCSVLine(l).map(s => s.trim()));
        let doubleableCount = 0;
        for (let col = 0; col < header.length; col++) {
            if (!header[col]) continue;
            let dancerCount = 0;
            for (const cells of dataRows) {
                if (cells[col]) dancerCount++;
            }
            if (dancerCount >= 1 && dancerCount <= 2) doubleableCount++;
        }
        const min = totalRoutines - Math.floor(doubleableCount / 2);
        return { min, max: totalRoutines };
    }

    // Clamp numSlots to valid range whenever the range changes
    $effect(() => {
        const { min, max } = slotRange;
        if (numSlots < min) numSlots = min;
        else if (numSlots > max) numSlots = max;
    });

    let maxIntermissionTolerance = $derived(Math.floor(slotRange.max / 2));

    function normalizeCsv(csv: string, aliases: Map<string, string>): string {
        if (!csv.trim()) return csv;
        return csv
            .trim()
            .split(/\r?\n/)
            .map((line, i) => {
                return parseCSVLine(line)
                    .map((cell, col) => {
                        const name = normalizeName(cell);
                        if (i === 0) return name; // header: just normalize
                        return aliases.get(name) ?? name; // body: normalize then alias-replace
                    })
                    .join(",");
            })
            .join("\n");
    }

    // ── Routine names (for constraint dropdowns) ──────────────────────────────
    let routineNames = $derived(extractRoutineNames(normalizedCsv));

    function extractRoutineNames(csv: string): string[] {
        if (!csv.trim()) return [];
        const firstLine = csv.trim().split(/\r?\n/)[0];
        const names = parseCSVLine(firstLine).map(normalizeName).filter(Boolean);
        // [Intermission] is appended by the WASM parse_csv; include it here for the constraint UI
        return [...names, '[Intermission]'];
    }

    // ── Constraint satisfaction (checked against the current displayed schedule) ─
    let constraintSatisfied = $derived(checkConstraints(customConstraints, slots));

    function checkConstraints(constraints: CustomConstraint[], currentSlots: SlotResult[]): (boolean | null)[] {
        // null = no schedule to evaluate against yet
        if (currentSlots.length === 0) return constraints.map(() => null);
        return constraints.map(c => {
            if (c.kind === 'in_slot') {
                const targetSlot = currentSlots.find(s => s.slot_number === c.slot);
                return targetSlot?.routines.includes(c.routine) ?? false;
            } else {
                // directly_before: c.afterRoutine must be in the slot immediately after c.beforeRoutine
                for (const slot of currentSlots) {
                    if (slot.routines.includes(c.beforeRoutine)) {
                        const next = currentSlots.find(s => s.slot_number === slot.slot_number + 1);
                        if (next?.routines.includes(c.afterRoutine)) return true;
                    }
                }
                return false;
            }
        });
    }

    // ── Auto-restart optimizer when inputs change ─────────────────────────────
    $effect(() => {
        const csv = normalizedCsv;
        const ns = numSlots;
        const it = intermissionTolerance;
        const constraints = customConstraints; // tracked so constraint changes also restart
        if (!csv.trim()) {
            worker?.terminate();
            worker = null;
            running = false;
            hasStarted = false;
            slots = [];
            score = null;
            status = "";
            return;
        }
        startOptimizer(csv, ns, it, constraints);
    });

    function startOptimizer(csv: string, ns: number, it: number, constraints: CustomConstraint[]) {
        worker?.terminate();

        running = true;
        hasStarted = true;
        status = "Optimizing…";
        slots = [];
        score = "";
        error = "";

        worker = new OptimizerWorker();

        worker.onmessage = (e: MessageEvent) => {
            if (e.data.type === "progress") {
                const result: OptimizeResult = JSON.parse(e.data.resultJson);
                const [d1, d2, mid] = result.score;
                score = { d1, d2, mid };
                slots = result.slots;
                status = "Optimizing…";
                if (activeTab !== "schedule") scheduleHasNewResult = true;
            } else if (e.data.type === "done") {
                running = false;
                status = "Done.";
                worker = null;
            } else if (e.data.type === "error") {
                running = false;
                status = "Error.";
                error = e.data.message;
                activeTab = "schedule";
                worker = null;
            }
        };

        worker.onerror = (e) => {
            running = false;
            status = "Error.";
            error = e.message ?? "Unknown worker error.";
            activeTab = "schedule";
            worker = null;
        };

        worker.postMessage({
            csvText: csv.trim(),
            numSlots: ns,
            intermissionTolerance: it,
            constraintsJson: JSON.stringify(constraints),
        });
    }

    // ── Obvious auto-merges ───────────────────────────────────────────────────
    /**
     * Scan all names in the CSV for groups that differ only in case and/or
     * punctuation (e.g. "O'Neal" vs "Oneal", "ABBY" vs "Abby").
     * Returns an alias map seeding those groups with the name that has
     * the most capital letters as canonical (ties: longer name wins).
     */
    function computeAutoMerges(csv: string): Map<string, string> {
        const result = new Map<string, string>();
        if (!csv.trim()) return result;

        const lines = csv.trim().split(/\r?\n/);
        const allNames = new Set<string>();

        for (const line of lines) {
            for (const cell of parseCSVLine(line)) {
                const name = normalizeName(cell);
                if (name) allNames.add(name);
            }
        }

        // Group names by lowercase + punctuation-stripped form
        const groups = new Map<string, string[]>();
        for (const name of allNames) {
            const key = name.toLowerCase().replace(/[^a-z0-9 ]/g, "");
            if (!groups.has(key)) groups.set(key, []);
            groups.get(key)!.push(name);
        }

        for (const group of groups.values()) {
            if (group.length <= 1) continue;
            // Canonical = most capital letters; tie-break: longest; tie-break: lexically first
            const canonical = group.reduce((best, name) => {
                const bc = (best.match(/[A-Z]/g) ?? []).length;
                const nc = (name.match(/[A-Z]/g) ?? []).length;
                if (nc !== bc) return nc > bc ? name : best;
                if (name.length !== best.length) return name.length > best.length ? name : best;
                return name < best ? name : best;
            });
            for (const name of group) {
                if (name !== canonical) result.set(name, canonical);
            }
        }

        return result;
    }

    // ── File loading ──────────────────────────────────────────────────────────
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
            const csv = (e.target?.result as string) ?? "";
            csvText = csv;
            fileName = file.name;
            aliasMap = computeAutoMerges(csv);
            numSlots = slotRange.max;
            dismissedSuggestions = new Set();
            customConstraints = [];
            scheduleHasNewResult = false;
            error = "";
            activeTab = "performers";
        };
        reader.readAsText(file);
    }

    function handleAliasMapChange(newMap: Map<string, string>) {
        aliasMap = newMap;
    }

    function handleConstraintsChange(newConstraints: CustomConstraint[]) {
        customConstraints = newConstraints;
    }

    function handleRegenerate() {
        if (normalizedCsv.trim()) startOptimizer(normalizedCsv, numSlots, intermissionTolerance, customConstraints);
    }
</script>

{#if isMobile}
    <div class="app-shell app-shell--mobile">
        <nav class="tab-bar">
            <button
                class={["tab-btn", activeTab === "setup" && "active"].filter(Boolean).join(" ")}
                onclick={() => activeTab = "setup"}
            >
                Setup
            </button>
            <button
                class={["tab-btn", activeTab === "performers" && "active"].filter(Boolean).join(" ")}
                onclick={() => activeTab = "performers"}
            >
                Performers
            </button>
            <button
                class={["tab-btn", activeTab === "schedule" && "active"].filter(Boolean).join(" ")}
                onclick={() => { activeTab = "schedule"; scheduleHasNewResult = false; }}
            >
                Schedule
                {#if scheduleHasNewResult}
                    <span class="tab-badge"></span>
                {:else if running && slots.length > 0}
                    <span class="tab-pulse"></span>
                {/if}
            </button>
        </nav>

        <div class="tab-content">
            <div class="tab-panel" class:tab-hidden={activeTab !== "setup"}>
                <Sidebar
                    {fileName}
                    bind:numSlots
                    minSlots={slotRange.min}
                    maxSlots={slotRange.max}
                    bind:intermissionTolerance
                    {maxIntermissionTolerance}
                    {status}
                    {score}
                    {running}
                    {dragOver}
                    variant="tab"
                    {routineNames}
                    {customConstraints}
                    {constraintSatisfied}
                    onFile={loadFile}
                    onDragOver={() => dragOver = true}
                    onDragLeave={() => dragOver = false}
                    onRegenerate={handleRegenerate}
                    onConstraintsChange={handleConstraintsChange}
                />
            </div>
            <div class="tab-panel" class:tab-hidden={activeTab !== "performers"}>
                {#if csvText}
                    <PerformersTab
                        {performerRoutines}
                        {aliasMap}
                        {dismissedSuggestions}
                        onAliasMapChange={handleAliasMapChange}
                        onDismissedChange={(s) => dismissedSuggestions = s}
                        onGoToSchedule={() => { activeTab = "schedule"; scheduleHasNewResult = false; }}
                    />
                {:else}
                    <NoFile />
                {/if}
            </div>
            <div class="tab-panel" class:tab-hidden={activeTab !== "schedule"}>
                <ScheduleTab {slots} {hasStarted} {error} />
            </div>
        </div>
    </div>
{:else}
    <div class="app-shell">
        <Sidebar
            {fileName}
            bind:numSlots
            minSlots={slotRange.min}
            maxSlots={slotRange.max}
            bind:intermissionTolerance
            {maxIntermissionTolerance}
            {status}
            {score}
            {running}
            {dragOver}
            {routineNames}
            {customConstraints}
            {constraintSatisfied}
            onFile={loadFile}
            onDragOver={() => dragOver = true}
            onDragLeave={() => dragOver = false}
            onRegenerate={handleRegenerate}
            onConstraintsChange={handleConstraintsChange}
        />

        <div class="main-area">
            {#if csvText}
                <nav class="tab-bar">
                    <button
                        class={["tab-btn", activeTab === "performers" && "active"].filter(Boolean).join(" ")}
                        onclick={() => activeTab = "performers"}
                    >
                        Performers
                    </button>
                    <button
                        class={["tab-btn", activeTab === "schedule" && "active"].filter(Boolean).join(" ")}
                        onclick={() => { activeTab = "schedule"; scheduleHasNewResult = false; }}
                    >
                        Optimized Schedule
                        {#if scheduleHasNewResult}
                            <span class="tab-badge"></span>
                        {:else if running && slots.length > 0}
                            <span class="tab-pulse"></span>
                        {/if}
                    </button>
                </nav>

                <div class="tab-content">
                    <div class="tab-panel" class:tab-hidden={activeTab !== "performers"}>
                        <PerformersTab
                            {performerRoutines}
                            {aliasMap}
                            {dismissedSuggestions}
                            onAliasMapChange={handleAliasMapChange}
                            onDismissedChange={(s) => dismissedSuggestions = s}
                            onGoToSchedule={() => { activeTab = "schedule"; scheduleHasNewResult = false; }}
                        />
                    </div>
                    <div class="tab-panel" class:tab-hidden={activeTab !== "schedule"}>
                        <ScheduleTab {slots} {hasStarted} {error} />
                    </div>
                </div>
            {:else}
                <div class="no-file">
                    <p class="no-file-prompt">Select a CSV to get started.</p>
                    <p class="no-file-privacy">No data leaves your device – everything runs locally in your browser.</p>
                </div>
            {/if}
        </div>
    </div>
{/if}

<style>
    .app-shell {
        display: flex;
        height: 100vh;
        overflow: hidden;
    }

    .main-area {
        flex: 1;
        display: flex;
        flex-direction: column;
        overflow: hidden;
        background: var(--color-bg);
    }

    .tab-bar {
        display: flex;
        gap: 0;
        border-bottom: 1px solid var(--color-border);
        background: var(--color-panel);
        flex-shrink: 0;
    }

    .tab-btn {
        position: relative;
        padding: 0.75rem 1.25rem;
        background: none;
        border: none;
        border-bottom: 2px solid transparent;
        color: var(--color-text-muted);
        font-size: 0.875rem;
        font-weight: 500;
        cursor: pointer;
        transition: color 0.15s, border-color 0.15s;
        display: flex;
        align-items: center;
        gap: 0.4rem;
    }

    .tab-btn:hover {
        color: var(--color-text);
    }

    .tab-btn.active {
        color: var(--color-accent-light);
        border-bottom-color: var(--color-accent);
    }

    .tab-pulse {
        width: 0.45rem;
        height: 0.45rem;
        border-radius: 50%;
        background: var(--color-accent);
        animation: pulse 1.2s ease-in-out infinite;
    }

    .tab-badge {
        width: 0.45rem;
        height: 0.45rem;
        border-radius: 50%;
        background: var(--color-success);
    }

    @keyframes pulse {
        0%, 100% { opacity: 1; transform: scale(1); }
        50% { opacity: 0.4; transform: scale(0.7); }
    }

    .tab-content {
        flex: 1;
        position: relative;
        overflow: hidden;
    }

    .tab-panel {
        position: absolute;
        inset: 0;
        overflow-y: auto;
    }

    .tab-hidden {
        visibility: hidden;
        pointer-events: none;
    }


    /* ── Mobile: 3-tab layout (Setup / Performers / Schedule) ───────────────── */
    .app-shell--mobile {
        flex-direction: column;
        height: auto;
        min-height: 100vh;
        overflow: visible;
    }

    .app-shell--mobile .tab-bar {
        position: sticky;
        top: 0;
        z-index: 10;
    }

    .app-shell--mobile .tab-content {
        flex: 1;
        position: static;
        overflow: visible;
    }

    .app-shell--mobile .tab-panel {
        position: static;
        overflow: visible;
    }

    .app-shell--mobile .tab-hidden {
        display: none;
    }
</style>
