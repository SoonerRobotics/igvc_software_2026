<script>
    import { Camera } from "lucide-svelte";

</script>
<div class="flex flex-col h-full bg-white rounded-lg border border-neutral overflow-hidden">
    <div class="flex-1 bg-black flex items-center justify-center min-h-0 relative">
        <div class="text-gray-600 text-center">
            <Camera class="w-16 h-16 mx-auto mb-2 opacity-50" />
            <p class="text-sm">Camera Placeholder</p>
        </div>

        <div class="absolute top-3 bright-3 bg-black/70 px-2 py-1 rounded text-xs font-mono text-white">
            RAW
        </div>
    </div>

    <div class="bg-white border-t border-neutral p-3 flex flex-wrap items-center gap-2">

    </div>
</div>