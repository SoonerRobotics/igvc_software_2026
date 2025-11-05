<script lang="ts">
	import '../app.css';
	import favicon from '$lib/assets/favicon.svg';
    import MainLayout from '../components/main-layout.svelte';
	
	let { children } = $props();
</script>

<svelte:head>
	<link rel="icon" href={favicon} />
    <title>SCR IGVC</title>
</svelte:head>

<MainLayout>
	{@render children()}
</MainLayout>
