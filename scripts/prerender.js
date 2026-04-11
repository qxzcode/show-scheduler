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

// Compile App.svelte for server-side rendering
const source = readFileSync(resolve(root, 'src/App.svelte'), 'utf-8');
const { js } = compile(source, {
    generate: 'server',
    runes: true,
    filename: 'src/App.svelte',
    css: 'injected',
});

// Strip import/export statements — we provide dependencies as function params.
let code = js.code;
code = code.replace(/^import\s+.*$/gm, '');
code = code.replace(/^export default\s+/m, '');

// Evaluate: wrap in a function that receives dependencies and returns the component.
const factory = new Function(
    '$', 'init', 'optimize', 'parse_csv',
    `"use strict";\n${code}\nreturn App;`,
);

// Stubs for WASM functions — never called during SSR due to `typeof window` guard.
const noop = () => {};
const App = factory(svelteServerInternals, () => Promise.resolve(), noop, noop);

// Render to HTML
const { head, body } = render(App, {});

// Inject into dist/index.html
let html = readFileSync(resolve(root, 'dist/index.html'), 'utf-8');
if (head) html = html.replace('</head>', `    ${head}\n    </head>`);
html = html.replace('<body>', `<body>\n        ${body}`);
writeFileSync(resolve(root, 'dist/index.html'), html);

console.log('Prerendered dist/index.html');
