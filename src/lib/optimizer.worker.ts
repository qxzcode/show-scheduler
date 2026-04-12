import init, { parse_csv, optimize_streaming } from '$lib/wasm/show_scheduler.js';

const wasmReady = init();

self.onmessage = async (e: MessageEvent<{ csvText: string; numSlots: number; intermissionTolerance: number; constraintsJson: string }>) => {
    const { csvText, numSlots, intermissionTolerance, constraintsJson } = e.data;
    try {
        await wasmReady;
        const routinesJson = parse_csv(csvText);
        optimize_streaming(routinesJson, numSlots, intermissionTolerance, constraintsJson, (resultJson: string) => {
            self.postMessage({ type: 'progress', resultJson });
        });
        self.postMessage({ type: 'done' });
    } catch (err) {
        self.postMessage({ type: 'error', message: String(err) });
    }
};
