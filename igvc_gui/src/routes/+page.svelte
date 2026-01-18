<script lang="ts">
    import { ImageFrame } from "$lib/arc/messages/messages/image-frame";
    import decoders from "$lib/arc/protocol/decoders";
    import { MessageType } from "$lib/arc/protocol/types";
    import { callCommand, connectionStatus, subscribe } from "$lib/arc/socket";
    import { toast } from "svelte-sonner";
    import ImageView from "../components/image-view.svelte";
    import CameraView from "../components/camera-view.svelte";

    subscribe<ImageFrame>(
        MessageType.ImageFrame,
        decoders.image_frame,
        (frame) => {},
    );

    $effect(() => {
        const toastid = toast.promise(
            () => {
                return new Promise<void>((resolve) => {
                    const unsubscribe = connectionStatus.subscribe((status) => {
                        if (status === "open") {
                            resolve();
                            unsubscribe();
                        }
                    });
                });
            },
            {
                loading: "Connecting to robot...",
                success: "Connected to robot!",
                error: "Failed to connect to robot.",
            },
        );

        return () => {
            toast.dismiss(toastid);
        };
    });
</script>

<div class="flex-1 overflow-auto p-4">
    <!-- <p>Connection status: {$connectionStatus}</p>

    <div>
        <h2>Camera Feeds</h2>

        <div class="flex flex-row gap-2">
            <button
                class="bg-red-500 text-white px-4 py-2 rounded"
                onclick={() => {
                    toast.promise(
                        callCommand(0x01),
                        {
                            loading: "Starting calibration...",
                            success: "Calibration started!",
                            error: (err) => `Failed to start calibration: ${err}`,
                        }
                    );
                }}
            >
                Start Calibration
            </button>

            <button
                class="bg-red-500 text-white px-4 py-2 rounded"
                onclick={() => {
                    toast.promise(
                        callCommand(0x02),
                        {
                            loading: "Stopping calibration...",
                            success: "Calibration stopped!",
                            error: (err) => `Failed to stop calibration: ${err}`,
                        }
                    );
                }}
            >
                Stop Calibration
            </button>

            <button
                class="bg-red-500 text-white px-4 py-2 rounded"
                onclick={() => {
                    toast.promise(
                        callCommand(0x03),
                        {
                            loading: "Saving calibration...",
                            success: "Calibration saved!",
                            error: (err) => `Failed to save calibration: ${err}`,
                        }
                    );
                }}
            >
                Save Calibration
            </button>
        </div>
    </div> -->

    <CameraView />
</div>
