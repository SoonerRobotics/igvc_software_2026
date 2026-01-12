<script lang="ts">
    import { ImageFrame } from "$lib/arc/messages/messages/image-frame";
    import decoders from "$lib/arc/protocol/decoders";
    import { MessageType } from "$lib/arc/protocol/types";
    import { connectionStatus, subscribe } from "$lib/arc/socket";

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

        {#if $connectionStatus == "open"}
            <div class="w-full max-h-160 flex flex-row gap-4">
                <!-- <ImageView id="front_view" />
                <ImageView id="hsv_view" />
                <ImageView id="yolo_view" /> -->
                <img src="http://localhost:8081/front_view" alt="front view" />
                <img src="http://localhost:8081/hsv_view" alt="hsv view" />
                <img src="http://localhost:8081/yolo_view" alt="yolo view" />
            </div>
        {/if}
    </div>
</div>
