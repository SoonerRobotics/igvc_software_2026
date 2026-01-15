<script lang="ts">

    const { id, alt } = $props();
</script>

<div class="max-w-[640px] max-h-[480px] min-w-[640px] min-h-[480px] bg-gray-800 rounded overflow-hidden p-2">
    <img src={`http://localhost:8081/${id}`} alt={alt} class="w-full h-full object-cover bg-gray-600" />
</div>
