<script lang="ts">
    import Camera from "@lucide/svelte/icons/camera";
    import ChevronDown from "@lucide/svelte/icons/chevron-down";
    import Columns2 from "@lucide/svelte/icons/columns-2";
    import Square from "@lucide/svelte/icons/square";

    type ViewMode = "single" | "split";

    const cameraViews = [
        { id: "front_view", label: "Front" },
        { id: "hsv_view", label: "HSV" },
    ];

    let viewMode: ViewMode = "single";
    let camera1 = cameraViews[0];
    let camera2 = cameraViews[1];

    function cameraSrc(id: string) {
        return `http://localhost:8081/${id}`;
    }
</script>

<div class="flex-1 rounded-lg border bg-card">
    <div class="flex items-center justify-between px-4 py-2 border-b">
        <h3 class="text-sm font-medium flex items-center gap-2">
            <Camera class="h-4 w-4 text-primary" />
            Camera Feed
        </h3>

        <div class="relative group">
            <button
                class="inline-flex items-center gap-1 h-8 px-2 border rounded-md text-sm bg-transparent"
            >
                {#if viewMode === "single"}
                    <Square class="h-3.5 w-3.5" />
                    Single
                {:else}
                    <Columns2 class="h-3.5 w-3.5" />
                    Split
                {/if}
                <ChevronDown class="h-3.5 w-3.5" />
            </button>

            <div
                class="absolute right-0 mt-1 w-32 rounded-md border bg-popover shadow
               opacity-0 group-hover:opacity-100 transition z-30"
            >
                <button
                    class="flex w-full items-center gap-2 px-3 py-2 text-sm hover:bg-muted"
                    on:click={() => (viewMode = "single")}
                >
                    <Square class="h-4 w-4" /> Single View
                </button>
                <button
                    class="flex w-full items-center gap-2 px-3 py-2 text-sm hover:bg-muted"
                    on:click={() => (viewMode = "split")}
                >
                    <Columns2 class="h-4 w-4" /> Split View
                </button>
            </div>
        </div>
    </div>

    <div class="p-4">
        {#if viewMode === "single"}
            <div class="space-y-2">
                <select
                    class="w-32 border rounded-md px-2 py-1 text-sm bg-background"
                    bind:value={camera1}
                >
                    {#each cameraViews as cam}
                        <option value={cam}>{cam.label}</option>
                    {/each}
                </select>

                <div
                    class="relative aspect-video rounded-md border overflow-hidden bg-black"
                >
                    <img
                        src={cameraSrc(camera1.id)}
                        alt={camera1.label}
                        class="absolute inset-0 w-full h-full object-cover"
                    />
                </div>
            </div>
        {:else}
            <div class="grid grid-cols-2 gap-4">
                <div class="space-y-2">
                    <select
                        class="w-full border rounded-md px-2 py-1 text-sm bg-background"
                        bind:value={camera1}
                    >
                        {#each cameraViews as cam}
                            <option value={cam}>{cam.label}</option>
                        {/each}
                    </select>

                    <div
                        class="relative aspect-video rounded-md border overflow-hidden bg-black"
                    >
                        <img
                            src={cameraSrc(camera1.id)}
                            alt={camera1.label}
                            class="absolute inset-0 w-full h-full object-cover"
                        />
                    </div>
                </div>

                <div class="space-y-2">
                    <select
                        class="w-full border rounded-md px-2 py-1 text-sm bg-background"
                        bind:value={camera2}
                    >
                        {#each cameraViews as cam}
                            <option value={cam}>{cam.label}</option>
                        {/each}
                    </select>

                    <div
                        class="relative aspect-video rounded-md border overflow-hidden bg-black"
                    >
                        <img
                            src={cameraSrc(camera2.id)}
                            alt={camera2.label}
                            class="absolute inset-0 w-full h-full object-cover"
                        />
                    </div>
                </div>
            </div>
        {/if}
    </div>
</div>
