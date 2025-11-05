<script lang="ts">
    import Navbar from "./navbar.svelte";
    import Sidebar from "./sidebar.svelte";

	let { children } = $props();
</script>

<div class="flex h-screen bg-background">
    <Sidebar />
    <div class="flex-1 flex flex-col overflow-hidden">
        <Navbar />
        <main class="flex-1 overflow-auto bg-white text-neutral p-4">
            {@render children()}
        </main>
    </div>
</div>
