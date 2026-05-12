<script lang="ts">
    import "./layout.css";

    import { onMount } from "svelte";
    import { connect } from "$lib/arc/connection";

    import StatusBar from "../components/statusbar.svelte";
    import Sidebar from "../components/sidebar.svelte";

    let { children } = $props();
    let sidebarCollapsed = $state(false);

    onMount(() => {
        connect();
    });
</script>

<div class="flex flex-col h-screen w-screen bg-slate-50 overflow-hidden">
    <StatusBar />

    <div class="flex flex-1 min-h-0 relative">
        <Sidebar collapsed={sidebarCollapsed} />

        <!-- Main content -->
        <main class="flex-1 min-w-0 min-h-0 p-2 overflow-auto bg-slate-50">
            {@render children()}
        </main>
    </div>
</div>
