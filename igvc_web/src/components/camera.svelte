<script lang="ts">
    const { label = "Camera", frame = null } = $props<{
        label?: string;
        frame?: Uint8Array | null;
    }>();

    let objectUrl = $state<string | null>(null);
    let prevFrame: Uint8Array | null = null;

    $effect(() => {
        if (frame === prevFrame) return;
        prevFrame = frame;

        // Revoke old URL to avoid memory leak
        if (objectUrl) {
            URL.revokeObjectURL(objectUrl);
            objectUrl = null;
        }

        if (frame) {
            const blob = new Blob([frame], { type: "image/jpeg" });
            objectUrl = URL.createObjectURL(blob);
        }
    });
</script>

<div class="flex flex-col gap-1 min-h-0">
    <span class="text-[10px] font-bold tracking-widest uppercase text-slate-400"
        >{label}</span
    >
    <div
        class="relative bg-slate-900 rounded-xl border-2 border-slate-200 overflow-hidden aspect-video w-full"
    >
        {#if objectUrl}
            <img
                src={objectUrl}
                alt={label}
                class="w-full h-full object-cover"
            />
        {:else}
            <div
                class="absolute inset-0 flex flex-col items-center justify-center gap-2"
            >
                <div
                    class="w-8 h-8 rounded-full border-2 border-slate-600 flex items-center justify-center"
                >
                    <span class="text-slate-600 text-lg">📷</span>
                </div>
                <span class="text-slate-600 text-xs font-mono">No signal</span>
            </div>
        {/if}

        <div class="absolute top-2 left-2 bg-black/60 rounded px-1.5 py-0.5">
            <span class="text-white text-[10px] font-mono font-bold"
                >{label}</span
            >
        </div>
    </div>
</div>
