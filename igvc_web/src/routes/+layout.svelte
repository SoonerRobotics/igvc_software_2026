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

        <!-- Collapse toggle tab -->
        <button
            onclick={() => (sidebarCollapsed = !sidebarCollapsed)}
            class="absolute bottom-6 z-10 w-5 h-10 bg-white border-2 border-l-0 border-slate-200
                   rounded-r-lg flex items-center justify-center text-slate-500
                   hover:bg-slate-100 hover:text-slate-900 transition-all duration-200
                   {sidebarCollapsed ? 'left-14' : 'left-52'}"
            title="Toggle sidebar"
        >
            <span class="text-xs font-bold">{sidebarCollapsed ? "›" : "‹"}</span
            >
        </button>

        <!-- Main content -->
        <main class="flex-1 min-w-0 min-h-0 p-2 overflow-auto bg-slate-50">
            {@render children()}
        </main>
    </div>
</div>
