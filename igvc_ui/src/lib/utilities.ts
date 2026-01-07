import type { ImageFrame } from "./flatbuffers/flatbuffers";

const canvasCache: { [id: string]: HTMLCanvasElement } = {};
let decoding = false;
export async function applyImageToCanvas(data: Uint8Array, id: string): Promise<void> {
    if (decoding) {
        return;
    }
    decoding = true;

    try {
        let canvas = canvasCache[id];
        if (!canvas) {
            canvas = document.getElementById(id) as HTMLCanvasElement;
            if (!canvas) {
                console.error(`Canvas with id ${id} not found`);
                return;
            }

            canvasCache[id] = canvas;
        }

        const ctx = canvas.getContext("2d"); // NOTE: Should we cache this?
        const bitmap = await createImageBitmap(
            new Blob([data.slice(0)], { type: "image/jpg" })
        );
        if (canvas.width !== bitmap.width || canvas.height !== bitmap.height) {
            canvas.width = bitmap.width;
            canvas.height = bitmap.height;
        }
        
        ctx?.drawImage(bitmap, 0, 0, canvas.width, canvas.height);
        bitmap.close();
    } finally {
        decoding = false;
    }
}

export function clearCache() {
    for (const key in canvasCache) {
        delete canvasCache[key];
    }
}