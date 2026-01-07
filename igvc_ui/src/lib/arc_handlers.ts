import type { ImageFrame } from "./flatbuffers/flatbuffers";
import { applyImageToCanvas } from "./utilities";


export function handleImageMessage(image: ImageFrame) {
    console.log(image);
    console.log(`Received image frame: ${image.width()}x${image.height()} of id ${image.identifier()} with data length ${image.imageDataLength()}`);

    const elementId = image.identifier();
    if (!elementId) {
        return;
    }

    applyImageToCanvas(image.imageDataArray()!, elementId);
}