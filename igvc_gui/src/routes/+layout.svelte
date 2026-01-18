<script lang="ts">
    import { connect, disconnect } from "$lib/arc/socket";
    import favicon from "$lib/assets/favicon.svg";
    import XOctagon from "@lucide/svelte/icons/octagon-x";
    import { Toaster } from "svelte-sonner";
    import "./layout.css";
    import Topbar from "../components/topbar.svelte";
    import Navbar from "../components/navbar.svelte";
    import ThemeProvider from "../components/theme-provider.svelte";

    let { children } = $props();

    $effect(() => {
        connect("ws://localhost:8080/");

        return () => {
            disconnect();
        };
    });
</script>

<svelte:head><link rel="icon" href={favicon} /></svelte:head>

<ThemeProvider>
    <Toaster richColors>
        {#snippet errorIcon()}
            <XOctagon class="w-6 h-6 text-red-500" />
        {/snippet}
    </Toaster>

    <div class="flex flex-col h-screen bg-background">
        <Topbar />
        <div class="flex flex-1 overflow-hidden">
            <Navbar />
            {@render children()}
        </div>
    </div>
</ThemeProvider>