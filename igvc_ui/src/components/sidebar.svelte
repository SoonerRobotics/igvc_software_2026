<script lang="ts">
    import { Cog, Camera, House, FolderCode, ChartLine, Map, Car } from "lucide-svelte";
    import { page } from "$app/state";
    const currentPage = page.url.pathname;
    const navigationItems = [
        {
            name: "Dashboard",
            href: "/",
            icon: House,
            active: currentPage === "/",
        },
        {
            name: "Vision",
            href: "/vision",
            icon: Camera,
            active: currentPage === "/vision/",
        },
        {
            name: "Graphs",
            href: "/graphs",  
            icon: ChartLine,
            active: currentPage === "/graphs/",
        },
        {
            name: "Mapping",
            href: "/mapping",
            icon: Map,
            active: currentPage === "/mapping/",
        },
        {
            name: "Self Drive",
            href: "/self-drive",
            icon: Car,
            active: currentPage === "/self-drive/",
        },
        {
            name: "Configuration",
            href: "/configuration",
            icon: Cog,
            active: currentPage === "/configuration/",
        },
        {
            name: "Logs",
            href: "/logs",
            icon: FolderCode,
            active: currentPage === "/logs/",
        },
    ];
</script>

<aside
    class="fixed md:static inset-y-16 left-0 w-48 bg-white border-r border-neutral z-40 md:z-auto flex flex-col"
>
    <div
        class="flex items-center justify-between px-4 py-4 border-b border-neutral min-h-16"
    >
        <h2 class="font-bold text-xl text-neutral">Twistopher</h2>
    </div>
    <nav class="flex-1 overflow-auto px-2 py-4 space-y-1">
        {#each navigationItems as item}
            <a
                href={item.href}
                class="flex items-center px-3 py-2 rounded text-neutral hover:bg-neutral/10 {item.active
                    ? 'bg-neutral/10 font-semibold'
                    : ''}"
            >
                <svelte:component this={item.icon} class="w-5 h-5 mr-3" />
                <span>{item.name}</span>
            </a>
        {/each}
    </nav>
</aside>
