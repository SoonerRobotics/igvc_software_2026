<script lang="ts">
    import { ImageFrame } from "$lib/arc/messages/messages/image-frame";
    import decoders from "$lib/arc/protocol/decoders";
    import { MessageType } from "$lib/arc/protocol/types";
    import { callCommand, connectionStatus, subscribe } from "$lib/arc/socket";

    subscribe<ImageFrame>(
        MessageType.ImageFrame,
        decoders.image_frame,
        (frame) => {},
    );
</script>

<div class="p-4 space-y-4">
    <p>Connection status: {$connectionStatus}</p>

    <div>
        <h2>Camera Feeds</h2>

        <div class="flex flex-row gap-2">
            <button
                class="bg-red-500 text-white px-4 py-2 rounded"
                on:click={() =>
                    callCommand(0x01)
                }
            >
                Start Calibration
            </button>

            <button
                class="bg-red-500 text-white px-4 py-2 rounded"
                on:click={() =>
                    callCommand(0x02)
                }
            >
                Stop Calibration
            </button>

            <button
                class="bg-red-500 text-white px-4 py-2 rounded"
                on:click={() =>
                    callCommand(0x03)
                }
            >
                Save Calibration
            </button>
        </div>

        {#if $connectionStatus == "open"}
            <div class="w-full max-h-160 grid grid-cols-2 gap-4 overflow-auto">
                <img src="http://localhost:8081/front_view" alt="front view" />
                <img src="http://localhost:8081/hsv_view" alt="hsv view" />
                <img src="http://localhost:8081/yolo_view" alt="yolo view" />
                <img src="http://localhost:8081/calibration_chessboard" alt="calibration chessboard" />
            </div>
        {/if}
    </div>
</div>
