<script lang="ts">
	import { connect, disconnect } from '$lib/arc/socket';
	import favicon from '$lib/assets/favicon.svg';
	import XOctagon from "@lucide/svelte/icons/octagon-x";
	import { Toaster } from 'svelte-sonner';
	import './layout.css';

	let { children } = $props();

    $effect(() => {
        connect("ws://localhost:8080/");

        return () => {
            disconnect();
        };
    });
</script>

<svelte:head><link rel="icon" href={favicon} /></svelte:head>
<Toaster richColors>
    {#snippet errorIcon()}
        <XOctagon class="w-6 h-6 text-red-500" />
    {/snippet}
</Toaster>
{@render children()}
