<script lang="ts">
	import './layout.css';
	import favicon from '$lib/assets/favicon.svg';
    import { connect, disconnect } from '$lib/arc/socket';

	let { children } = $props();

    $effect(() => {
        connect("ws://localhost:8080/");

        return () => {
            disconnect();
        };
    });
</script>

<svelte:head><link rel="icon" href={favicon} /></svelte:head>
{@render children()}
