import { svelte } from '@sveltejs/vite-plugin-svelte';
import { defineConfig } from 'vite';
import { fileURLToPath } from 'url';

export default defineConfig({
    base: process.env.BASE_PATH ?? '/',
    plugins: [svelte()],
    publicDir: 'static',
    resolve: {
        alias: {
            $lib: fileURLToPath(new URL('./src/lib', import.meta.url)),
        },
    },
});
