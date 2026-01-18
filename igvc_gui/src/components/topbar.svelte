<script lang="ts">
    import Bot from "@lucide/svelte/icons/bot";
    import Zap from "@lucide/svelte/icons/zap";
    import Compass from "@lucide/svelte/icons/compass";
    import Car from "@lucide/svelte/icons/car";
    import Sun from "@lucide/svelte/icons/sun";
    import Moon from "@lucide/svelte/icons/moon";
    import { theme, toggleTheme } from "$lib/theme";

    type RobotMode = "Disabled" | "Manual" | "Autonomous";
    type Mission = "Autonav" | "SelfDrive";

    const mode: RobotMode = "Autonomous";
    const mission: Mission = "SelfDrive";
    const mobilityEnabled = true;

    function getModeColor(mode: RobotMode) {
        switch (mode) {
            case "Disabled":
                return "bg-muted text-white";
            case "Manual":
                return "bg-warning text-white";
            case "Autonomous":
                return "bg-primary text-white";
        }
    }
</script>

<header
    class="h-14 border-b border-border bg-card px-4 flex items-center justify-between"
>
    <div class="flex items-center gap-3">
        <Bot class="h-6 w-6 text-primary" />
        <h1 class="text-lg font-semibold text-foreground">
            Suspended Disbelief
        </h1>
    </div>

    <div class="flex items-center gap-4">
        <div class="flex items-center gap-2">
            <span class="text-sm text-muted-foreground">Mode</span>
            <span
                class={`inline-flex rounded-md px-2 py-0.5 text-xs font-medium ${getModeColor(mode)}`}
            >
                {mode}
            </span>
        </div>

        <div class="flex items-center gap-2">
            <Compass class="h-4 w-4 text-muted-foreground" />
            <span class="text-sm text-muted-foreground">Mission</span>
            <span
                class="inline-flex rounded-md bg-secondary px-2 py-0.5 text-xs font-medium"
            >
                {mission}
            </span>
        </div>

        <div class="flex items-center gap-2">
            <Car class="h-4 w-4 text-muted-foreground" />
            <span class="text-sm text-muted-foreground">Mobility</span>

            <span
                class={`inline-flex items-center gap-1 rounded-md px-2 py-0.5 text-xs font-medium ${
                    mobilityEnabled
                        ? "bg-primary text-white"
                        : "bg-destructive text-white"
                }`}
            >
                {#if mobilityEnabled}
                    <Zap class="h-3 w-3" />
                    Enabled
                {:else}
                    Disabled
                {/if}
            </span>
        </div>

        <div class="relative group">
            <button
                class="h-8 w-8 inline-flex items-center justify-center rounded-md hover:bg-muted"
                on:click={toggleTheme}
                aria-label="Toggle theme"
            >
                {#if $theme === "dark"}
                    <Sun
                        class="h-4 w-4 text-muted-foreground group-hover:text-foreground"
                    />
                {:else}
                    <Moon
                        class="h-4 w-4 text-muted-foreground group-hover:text-foreground"
                    />
                {/if}
            </button>

            <div
                class="absolute top-full mt-1 left-1/2 -translate-x-1/2
               rounded bg-popover px-2 py-1 text-xs
               text-popover-foreground opacity-0
               group-hover:opacity-100 transition"
            >
                Switch to {$theme === "dark" ? "light" : "dark"} mode
            </div>
        </div>
    </div>
</header>
