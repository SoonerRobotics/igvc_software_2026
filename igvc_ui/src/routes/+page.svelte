<script>
    import { igvcSocket } from "$lib/igvc";
</script>

<p>IGVC Socket Status: {$igvcSocket.status == "connected" ? 'Connected' : 'Disconnected'}</p>
<p>IGVC Time: {$igvcSocket.status == "connected" ? $igvcSocket.time : 'N/A'}</p>