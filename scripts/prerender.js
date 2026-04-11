// Renders App.svelte to HTML at build time and injects it into dist/index.html
// so the browser has content immediately without waiting for JS.
//
// We can't use Vite's ssrLoadModule because Vite 8's rolldown parser rejects the
// Svelte 5 SSR compiler output (statement before imports — valid ES but rolldown
// is strict). Instead, compile directly with the Svelte compiler API and evaluate
// the result ourselves.
import { compile } from 'svelte/compiler';
import { render } from 'svelte/server';
import * as svelteServerInternals from 'svelte/internal/server';
import { readFileSync, writeFileSync } from 'fs';
import { resolve, dirname } from 'path';
import { fileURLToPath } from 'url';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');

/**
 * Compile a Svelte component for SSR, strip its imports (we supply deps
 * manually), and return the component function.
 *
 * @param {string} sourcePath - path relative to src/
 * @param {Record<string, unknown>} deps - values to inject for stripped imports
 */
function buildComponent(sourcePath, deps = {}) {
    const absolutePath = resolve(root, 'src', sourcePath);
    const source = readFileSync(absolutePath, 'utf-8');
    const { js } = compile(source, {
        generate: 'server',
        runes: true,
        filename: absolutePath,
        css: 'external',
    });

    let code = js.code;
    // Strip import statements — we provide all deps as function parameters.
    code = code.replace(/^import\s+.*$/gm, '');
    // Strip the `export default` so the name becomes a local variable.
    code = code.replace(/^export default\s+/m, '');
    // Derive the component name from what was exported.
    const nameMatch = code.match(/(?:function|class|const)\s+(\w+)/);
    const componentName = nameMatch?.[1] ?? 'default';

    const paramNames = Object.keys(deps);
    const paramValues = Object.values(deps);
    const factory = new Function('$', ...paramNames,
        `"use strict";\n${code}\nreturn ${componentName};`
    );
    return factory(svelteServerInternals, ...paramValues);
}

// ── Build components in dependency order ──────────────────────────────────────

// Sidebar: no component or module deps
const Sidebar = buildComponent('lib/Sidebar.svelte');

// ScheduleTab: no component or module deps
const ScheduleTab = buildComponent('lib/ScheduleTab.svelte');

// PerformersTab: imports suggestMerges from similarity.ts.
// During SSR csvText is always "" so PerformersTab is never rendered,
// but stub the import to avoid ReferenceErrors in compiled code.
const PerformersTab = buildComponent('lib/PerformersTab.svelte', {
    suggestMerges: () => [],
    jaroWinkler: () => 0,
    onGoToSchedule: () => {},
});

// App: imports the worker class and sub-components.
// The worker is never constructed during SSR (csvText === "").
class OptimizerWorkerStub {
    postMessage() {}
    terminate() {}
    set onmessage(_) {}
    set onerror(_) {}
}

const App = buildComponent('App.svelte', {
    OptimizerWorker: OptimizerWorkerStub,
    Sidebar,
    PerformersTab,
    ScheduleTab,
});

// ── Render to HTML ────────────────────────────────────────────────────────────
const { head, body } = render(App, {});

// ── Inject into dist/index.html ───────────────────────────────────────────────
let html = readFileSync(resolve(root, 'dist/index.html'), 'utf-8');
if (head) html = html.replace('</head>', `    ${head}\n    </head>`);
html = html.replace('<body>', `<body>\n        ${body}`);
writeFileSync(resolve(root, 'dist/index.html'), html);

console.log('Prerendered dist/index.html');
