<script lang="ts">
    /**
     *     enum DeviceState : uint8_t
    {
        OFF = 0,
        INITIALIZING = 1,
        READY = 2,
        OPERATING = 3,
        UNKNOWN = 4,
        ERRORED = 5,
    };
     */
    const DeviceState = {
        OFF: 0,
        INITIALIZING: 1,
        READY: 2,
        OPERATING: 3,
        UNKNOWN: 4,
        ERRORED: 5,
    } as const;

    const devices: {
        name: string;
        status: (typeof DeviceState)[keyof typeof DeviceState];
    }[] = [
        { name: "igvc_controller", status: DeviceState.READY },
        { name: "igvc_vision", status: DeviceState.OPERATING },
        { name: "igvc_hardware_can", status: DeviceState.OPERATING },
        { name: "igvc_hardware_gps", status: DeviceState.INITIALIZING },
        { name: "igvc_camera_front", status: DeviceState.ERRORED },
        { name: "igvc_camera_rear", status: DeviceState.READY },
    ];

    const getStatusColor = (
        status: (typeof DeviceState)[keyof typeof DeviceState],
    ) => {
        switch (status) {
            case DeviceState.OFF:
                return "bg-status-offline text-white";
            case DeviceState.INITIALIZING:
                return "bg-status-warning text-white";
            case DeviceState.READY:
                return "bg-status-ready text-white";
            case DeviceState.OPERATING:
                return "bg-status-ready text-white";
            case DeviceState.UNKNOWN:
                return "bg-status-error text-white";
            case DeviceState.ERRORED:
                return "bg-status-error text-white";
            default:
                return "bg-gray-500 text-white";
        }
    };

    const getStatusDot = (
        status: (typeof DeviceState)[keyof typeof DeviceState],
    ) => {
        switch (status) {
            case DeviceState.OFF:
                return "bg-status-offline";
            case DeviceState.INITIALIZING:
                return "bg-status-warning animate-pulse";
            case DeviceState.READY:
                return "bg-status-ready";
            case DeviceState.OPERATING:
                return "bg-status-ready";
            case DeviceState.UNKNOWN:
                return "bg-status-error";
            case DeviceState.ERRORED:
                return "bg-status-error";
            default:
                return "bg-gray-500";
        }
    };
</script>

<div
    class="bg-white rounded-lg border border-neutral flex flex-col h-full overflow-hidden"
>
    <div class="px-4 py-3 border-b border-neutral shrink-0">
        <h3 class="font-semibold text-foreground">Devices Status</h3>
        <p class="text-xs text-muted-foreground mt-1">
            {devices.filter((d) => d.status !== DeviceState.OFF).length}/{devices.length} active
        </p>
    </div>

    <div class="flex-1 overflow-auto">
        <div class="divide-y divide-border">
            {#each devices as device}
                <div class="px-4 py-3 hover:bg-secondary flex items-center justify-between gap-3">
                    <div class="flex items-center space-x-3">
                        <div class="w-3 h-3 rounded-full {getStatusDot(device.status)}"></div>
                        <span class="font-medium text-foreground">{device.name}</span>
                    </div>
                    <div
                        class={`px-2 py-1 rounded text-xs font-semibold shrink-0 whitespace-nowrap ${getStatusColor(device.status)}`}
                    >
                        {#if device.status === DeviceState.OFF}
                            OFF
                        {:else if device.status === DeviceState.INITIALIZING}
                            INITIALIZING
                        {:else if device.status === DeviceState.READY}
                            READY
                        {:else if device.status === DeviceState.OPERATING}
                            OPERATING
                        {:else if device.status === DeviceState.UNKNOWN}
                            UNKNOWN
                        {:else if device.status === DeviceState.ERRORED}
                            ERRORED
                        {/if}
                    </div>
                </div>
            {/each}
        </div>
    </div>
</div>
