<script>
    import { igvcSocket } from "$lib/igvc";
    import Dashboard from "../components/pages/dashboard/dashboard.svelte";

    igvcSocket.subscribe((data) => {

    });
</script>

<Dashboard />