<script>
    import CameraView from "./camera-view.svelte";
    import DeviceStates from "./device-states.svelte";

</script>
<div class="h-full flex flex-col gap-4 overflow-auto">
    <div class="flex flex-col lg:flex-row gap-4 flex-1 min-h-0">
        <div class="flex-1 flex flex-col min-w-0">
            <CameraView />
        </div>
        <div class="lg:w-80 shrink-0">
            <DeviceStates />
        </div>
    </div>
</div>