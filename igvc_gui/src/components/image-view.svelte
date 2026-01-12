<script lang="ts">
    import type { ImageFrame } from "$lib/arc/messages/messages/image-frame";
    import { VisionCapabilities } from "$lib/arc/protocol/capabilities";
    import { addVisionCapability } from "$lib/arc/protocol/commands/capabilities";
    import decoders from "$lib/arc/protocol/decoders";
    import { MessageType } from "$lib/arc/protocol/types";
    import { subscribe, unsubscribe } from "$lib/arc/socket";

    const {
        id,
        width,
        height,
    }: {
        id: string;
        width?: number;
        height?: number;
    } = $props();

    let canvas: HTMLCanvasElement;
    let ctx: CanvasRenderingContext2D | null = null;
    let subscription: any;

    let bitmap: ImageBitmap | null = null;
    async function drawFrame(frame: ImageFrame) {
        const bytes = frame.imageDataArray();
        if (bytes == null || bytes.length === 0) {
            return;
        }

        const blob = new Blob([bytes.slice(0)], { type: "image/jpeg" });

        bitmap?.close();
        bitmap = await createImageBitmap(blob);

        if (canvas.width !== bitmap.width || canvas.height !== bitmap.height) {
            canvas.width = bitmap.width;
            canvas.height = bitmap.height;
        }

        ctx?.drawImage(bitmap, 0, 0);
    }

    $effect(() => {
        ctx = canvas.getContext("2d", { alpha: false });

        subscription = subscribe(
            MessageType.ImageFrame,
            decoders.image_frame,
            (frame) => {
                drawFrame(frame);
            },
            (frame) => frame.identifier() === id,
        );

        // request capabilities
        addVisionCapability(VisionCapabilities.FrontCamera | VisionCapabilities.HsvView | VisionCapabilities.YoloView);

        return () => {
            unsubscribe(subscription);
            bitmap?.close();
        };
    });
</script>

<canvas
    bind:this={canvas}
    {width}
    {height}
    style="display: block; width: 100%; height: auto;"
    class="rounded-lg"
>
</canvas>
