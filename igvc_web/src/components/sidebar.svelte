<script lang="ts">
    import { page } from "$app/state";

    const { collapsed = false } = $props<{ collapsed?: boolean }>();

    const navItems = [
        { href: "/dashboard", label: "Dashboard", icon: "" },
        { href: "/camera", label: "Camera", icon: "" },
        { href: "/metrics", label: "Metrics", icon: "" },
        { href: "/hsv", label: "HSV Cal", icon: "" },
        { href: "/yolo", label: "YOLO", icon: "" },
        { href: "/map", label: "Map", icon: "" },
        { href: "/config", label: "Config", icon: "" },
    ];
</script>

<aside
    class="flex flex-col h-full bg-white border-r-2 border-slate-200 transition-all duration-200 {collapsed
        ? 'w-14'
        : 'w-52'} shrink-0"
>
    <nav class="flex flex-col gap-1 p-2 flex-1 overflow-y-auto">
        {#each navItems as item}
            {@const active = page.url.pathname.startsWith(item.href)}
            <a
                href={item.href}
                class="flex items-center gap-3 px-2 py-3 rounded-lg transition-all duration-100
                       {active
                    ? 'bg-red-400 text-white'
                    : 'text-slate-600 hover:bg-slate-100 hover:text-slate-900'}"
            >
                <span class="text-lg shrink-0 w-6 text-center leading-none"
                    >{item.icon}</span
                >
                {#if !collapsed}
                    <span
                        class="text-[13px] font-semibold tracking-wide truncate"
                    >
                        {item.label}
                    </span>
                {/if}
            </a>
        {/each}
    </nav>

    {#if !collapsed}
        <div class="px-4 py-3 border-t border-slate-200 shrink-0">
            <span
                class="text-[10px] text-slate-400 tracking-widest uppercase font-mono"
                >v0.1.0</span
            >
        </div>
    {/if}
</aside>
