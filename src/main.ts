import App from './App.svelte';
import { hydrate } from 'svelte';

hydrate(App, { target: document.body });
