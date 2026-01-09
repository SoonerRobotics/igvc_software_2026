import { filter, map, sampleTime, share, shareReplay } from "rxjs";
import { messages$ } from "../messages";
import { MessageType } from "../../types";
import decoder from "../../protocol/decoders";

const imageMessages$ = messages$.pipe(
    filter(m => m.type === MessageType.ImageFrame),
    map(m => decoder.image_frame(m)),
    share()
);

function createImageStream(image_id: string) {
    return imageMessages$.pipe(
        filter(m => m.identifier() == image_id),
        map(m => new Blob([m.imageDataArray()?.slice(0)!], { type: "image/jpeg" })),
        sampleTime(10),
        shareReplay(1)
    )
}

export const frontView$ = createImageStream("front_view");
export const rearView$ = createImageStream("rear_view");
export const yoloView$ = createImageStream("yolo_view");