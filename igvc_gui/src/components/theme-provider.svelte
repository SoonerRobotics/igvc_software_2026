<script lang="ts">
    import { onMount } from "svelte";
    import { theme, resolvedTheme } from "$lib/theme";
    import { browser } from "$app/environment";

    function applyTheme(value: "light" | "dark") {
        document.documentElement.classList.remove("light", "dark");
        document.documentElement.classList.add(value);
    }

    onMount(() => {
        const unsubscribe = resolvedTheme.subscribe((value) => {
            if (!browser) return;
            applyTheme(value);
        });

        return unsubscribe;
    });
</script>

<slot />
