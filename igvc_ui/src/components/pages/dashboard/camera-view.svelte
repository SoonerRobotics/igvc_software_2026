<script>
    import { Camera } from "lucide-svelte";

    let currentView = $state("front");
    let currentFilter = $state("raw");
    const views = ["front", "top"];
    const filters = ["raw", "filtered", "yolo"];
</script>

<div
    class="flex flex-col h-full bg-white rounded-lg border border-neutral overflow-hidden"
>
    <div
        class="flex-1 bg-black flex items-center justify-center min-h-0 relative"
    >
        <div class="text-gray-600 text-center">
            <Camera class="w-16 h-16 mx-auto mb-2 opacity-50" />
            <p class="text-sm">Camera Placeholder</p>
        </div>

        <div
            class="absolute top-3 bright-3 bg-black/70 px-2 py-1 rounded text-xs font-mono text-white"
        >
            {currentView.toLocaleUpperCase()} ({currentFilter.toLocaleUpperCase()})
        </div>
    </div>

    <div
        class="bg-white border-t border-neutral p-3 flex flex-wrap items-center gap-2"
    >
        <div class="flex gap-1 items-center">
            <details class="dropdown dropdown-top">
                <summary class="btn btn-ghost border border-neutral m-1">
                    View: {currentView}
                </summary>
                <ul class="menu dropdown-content bg-white rounded-box z-1 w-52 p-2 shadow-sm">
                    {#each views as view}
                        <li>
                            <a href="#" onclick={() => (currentView = view)}>{view}</a>
                        </li>
                    {/each}
                </ul>
            </details>
        </div>
        <div class="flex gap-1 items-center ml-auto">
            <details class="dropdown dropdown-top">
                <summary class="btn btn-ghost border border-neutral m-1">
                    Filter: {currentFilter}
                </summary>
                <ul class="menu dropdown-content bg-white rounded-box z-1 w-52 p-2 shadow-sm">
                    {#each filters as filter}
                        <li>
                            <a href="#" onclick={() => (currentFilter = filter)}>{filter}</a>
                        </li>
                    {/each}
                </ul>
            </details>
        </div>
    </div>
</div>
