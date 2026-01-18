<script lang="ts">
    import { derived } from "svelte/store";

    import LayoutDashboard from "@lucide/svelte/icons/layout-dashboard";
    import Wrench from "@lucide/svelte/icons/wrench";
    import Eye from "@lucide/svelte/icons/eye";
    import Settings from "@lucide/svelte/icons/settings";
    import FileText from "@lucide/svelte/icons/file-text";
    import ChevronLeft from "@lucide/svelte/icons/chevron-left";
    import ChevronRight from "@lucide/svelte/icons/chevron-right";
    import { page } from "$app/stores";
    import { cn } from "$lib/utils";

    let collapsed = false;
    const pathname = derived(page, ($page) => $page.url.pathname);

    const navItems = [
        { name: "Dashboard", href: "/", icon: LayoutDashboard },
        { name: "Tools", href: "/tools", icon: Wrench },
        { name: "Vision", href: "/vision", icon: Eye },
        { name: "Configuration", href: "/configuration", icon: Settings },
        { name: "Logs", href: "/logs", icon: FileText },
    ];
</script>

<aside
    class={cn(
        "h-full border-r border-border bg-sidebar flex flex-col transition-all duration-300",
        collapsed ? "w-16" : "w-56",
    )}
>
    <nav class="flex-1 py-4">
        <ul class="space-y-1 px-2">
            {#each navItems as item}
                {#if $pathname === item.href}
                    <li>
                        <a
                            href={item.href}
                            class="flex items-center gap-3 px-3 py-2.5 rounded-md text-sm font-medium bg-sidebar-accent text-sidebar-accent-foreground"
                        >
                            <svelte:component
                                this={item.icon}
                                class="h-5 w-5 shrink-0"
                            />
                            {#if !collapsed}
                                <span>{item.name}</span>
                            {/if}
                        </a>
                    </li>
                {:else}
                    <li class="relative group">
                        <a
                            href={item.href}
                            class="flex items-center gap-3 px-3 py-2.5 rounded-md text-sm font-medium text-sidebar-foreground/70 hover:bg-sidebar-accent/50 hover:text-sidebar-foreground"
                        >
                            <svelte:component
                                this={item.icon}
                                class="h-5 w-5 shrink-0"
                            />
                            {#if !collapsed}
                                <span>{item.name}</span>
                            {/if}
                        </a>

                        {#if collapsed}
                            <div
                                class="absolute left-full top-1/2 -translate-y-1/2 ml-2
                       rounded bg-popover px-2 py-1 text-xs text-popover-foreground
                       opacity-0 group-hover:opacity-100 transition"
                            >
                                {item.name}
                            </div>
                        {/if}
                    </li>
                {/if}
            {/each}
        </ul>
    </nav>

    <div class="p-2 border-t border-sidebar-border">
        <button
            class="w-full flex justify-center text-sidebar-foreground/70 hover:text-sidebar-foreground"
            on:click={() => (collapsed = !collapsed)}
        >
            {#if collapsed}
                <ChevronRight class="h-4 w-4" />
            {:else}
                <ChevronLeft class="h-4 w-4" />
            {/if}
        </button>
    </div>
</aside>
