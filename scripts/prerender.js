// Renders App.svelte to HTML at build time and injects it into dist/index.html
// so the browser has content immediately without waiting for JS.
//
// We can't use Vite's ssrLoadModule because Vite 8's rolldown parser rejects the
// Svelte 5 SSR compiler output (statement before imports — valid ES but rolldown
// is strict). Instead, compile directly with the Svelte compiler API and evaluate
// the result ourselves.
//
// buildComponent() auto-detects imports from each component's compiled SSR output:
//   - svelte/internal/server  → always the $ parameter (svelteServerInternals)
//   - other .svelte files     → recursively compiled via buildComponent()
//   - ?worker imports         → stubbed with a no-op class
//   - everything else         → stubbed with a recursive no-op Proxy
//
// Adding new Svelte components never requires editing this file.
import { compile } from 'svelte/compiler';
import { render } from 'svelte/server';
import * as svelteServerInternals from 'svelte/internal/server';
import { readFileSync, writeFileSync, existsSync } from 'fs';
import { resolve, dirname, extname } from 'path';
import { fileURLToPath } from 'url';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const srcDir = resolve(root, 'src');

// ── Stub for non-Svelte imports ───────────────────────────────────────────────
// Returns a recursive Proxy: every property access and every call returns
// another stub. Safe for any usage pattern in components that never render
// during SSR (e.g. PerformersTab when csvText === "").
function makeStub() {
    const fn = () => [];
    return new Proxy(fn, {
        get(_, key) {
            if (key === Symbol.toPrimitive || key === Symbol.iterator) return undefined;
            return makeStub();
        },
        apply: () => [],
    });
}

// ── Import parser ─────────────────────────────────────────────────────────────
// Parses the single-line import statements emitted by the Svelte SSR compiler.
function parseImports(code) {
    const imports = [];
    const re = /^import\s+(.+?)\s+from\s+["']([^"']+)["'];?\s*$/gm;
    let m;
    while ((m = re.exec(code)) !== null) {
        const [, clause, path] = m;
        if (clause.trim().startsWith('type ')) continue; // import type { ... }

        const names = [];

        // namespace: * as Foo
        const starM = clause.match(/\*\s+as\s+(\w+)/);
        if (starM) {
            names.push(starM[1]);
        } else {
            // default import (word before a comma or end)
            const defM = clause.match(/^(\w+)(?:\s*,|\s*$)/);
            if (defM) names.push(defM[1]);
        }

        // named imports: { a, b as c, type d }
        const namedM = clause.match(/\{([^}]+)\}/);
        if (namedM) {
            for (const part of namedM[1].split(',')) {
                const t = part.trim();
                if (!t || t.startsWith('type ')) continue;
                const asM = t.match(/\w+\s+as\s+(\w+)/);
                names.push(asM ? asM[1] : t.split(/\s+/)[0]);
            }
        }

        if (names.length) imports.push({ path, names });
    }
    return imports;
}

// ── Path resolver ─────────────────────────────────────────────────────────────
function resolveImport(importPath, fromFile) {
    const stripped = importPath.replace(/\?.*$/, ''); // drop ?worker etc.
    let abs;
    if (stripped.startsWith('$lib/')) {
        abs = resolve(srcDir, 'lib', stripped.slice(5));
    } else if (stripped.startsWith('.')) {
        abs = resolve(dirname(fromFile), stripped);
    } else {
        return null; // external package (svelte/*, etc.)
    }
    if (existsSync(abs)) return abs;
    if (existsSync(abs + '.ts')) return abs + '.ts';
    if (existsSync(abs.replace(/\.js$/, '.ts'))) return abs.replace(/\.js$/, '.ts');
    return abs;
}

// ── Component builder ─────────────────────────────────────────────────────────
const cache = new Map();

/**
 * Compile a Svelte component for SSR and return the component function.
 * Imports are auto-detected and resolved; no manual dep listing needed.
 *
 * @param {string} sourcePath - path relative to src/
 */
function buildComponent(sourcePath) {
    if (cache.has(sourcePath)) return cache.get(sourcePath);

    const absolutePath = resolve(srcDir, sourcePath);
    const source = readFileSync(absolutePath, 'utf-8');
    const { js } = compile(source, {
        generate: 'server',
        runes: true,
        filename: absolutePath,
        css: 'external',
    });

    let code = js.code;

    // Resolve imports before stripping them from the code
    const deps = {};
    for (const { path, names } of parseImports(code)) {
        if (path === 'svelte/internal/server') continue; // provided as $

        const isWorker = path.includes('?worker');
        const resolved = isWorker ? null : resolveImport(path, absolutePath);

        let value;
        if (isWorker) {
            value = class {
                postMessage() {} terminate() {}
                set onmessage(_) {} set onerror(_) {}
            };
        } else if (resolved && extname(resolved) === '.svelte') {
            value = buildComponent(resolved.slice(srcDir.length + 1));
        } else {
            value = makeStub(); // TS utility, external lib, etc.
        }

        for (const name of names) deps[name] = value;
    }

    // Strip imports and export; eval the rest as a factory function
    code = code.replace(/^import\s+.*$/gm, '');
    code = code.replace(/^export default\s+/m, '');
    // import.meta is unavailable in new Function() (non-module context); stub it out.
    // These expressions are only reached at runtime (e.g. click handlers), never during SSR render.
    code = code.replace(/import\.meta\.env/g, '({BASE_URL:"/",MODE:"production",DEV:false,PROD:true,SSR:false})');
    const nameMatch = code.match(/(?:function|class|const)\s+(\w+)/);
    const componentName = nameMatch?.[1] ?? 'default';

    const paramNames = Object.keys(deps);
    const paramValues = Object.values(deps);
    const factory = new Function('$', ...paramNames,
        `"use strict";\n${code}\nreturn ${componentName};`
    );
    const component = factory(svelteServerInternals, ...paramValues);
    cache.set(sourcePath, component);
    return component;
}

// ── Render to HTML ────────────────────────────────────────────────────────────
const { head, body } = render(buildComponent('App.svelte'), {});

// ── Inject into dist/index.html ───────────────────────────────────────────────
let html = readFileSync(resolve(root, 'dist/index.html'), 'utf-8');
if (head) html = html.replace('</head>', `    ${head}\n    </head>`);
html = html.replace('<body>', `<body>\n        ${body}`);
writeFileSync(resolve(root, 'dist/index.html'), html);

console.log('Prerendered dist/index.html');
