<script lang="ts">
    import {
        connectionStatus,
        wsUrl,
        connect,
        disconnect,
        isConnected,
    } from "$lib/arc/connection";

    let editingUrl = $state(false);
    let urlInput = $state("");

    function toggleConnection() {
        if ($isConnected) {
            disconnect();
        } else {
            connect();
        }
    }

    function startEditUrl() {
        urlInput = $wsUrl;
        editingUrl = true;
    }

    function saveUrl(e?: KeyboardEvent) {
        if (e && e.key !== "Enter") return;
        wsUrl.set(urlInput);
        editingUrl = false;
        if ($isConnected) {
            disconnect();
            setTimeout(() => connect(), 300);
        }
    }

    const statusDot: Record<string, string> = {
        connected: "bg-emerald-500",
        connecting: "bg-amber-400 animate-pulse",
        disconnected: "bg-slate-400",
        error: "bg-red-500",
    };

    const statusLabel: Record<string, string> = {
        connected: "Connected",
        connecting: "Connecting…",
        disconnected: "Offline",
        error: "Error",
    };

    const statusText: Record<string, string> = {
        connected: "text-emerald-700",
        connecting: "text-amber-600",
        disconnected: "text-slate-500",
        error: "text-red-600",
    };
</script>

<header
    class="flex items-center gap-4 px-4 h-12 bg-white border-b-2 border-slate-200 shrink-0"
>
    <span class="text-sm font-black tracking-widest text-slate-900 uppercase shrink-0">IGVC</span>

    <div class="w-px h-5 bg-slate-200"></div>

    <div class="flex items-center gap-2 shrink-0">
        <div
            class="w-2.5 h-2.5 rounded-full {statusDot[$connectionStatus]}"
        ></div>
        <span class="text-sm font-semibold {statusText[$connectionStatus]}">
            {statusLabel[$connectionStatus]}
        </span>
    </div>

    <div class="w-px h-5 bg-slate-200"></div>

    <div class="flex items-center gap-2 flex-1 min-w-0">
        {#if editingUrl}
            <input
                type="text"
                bind:value={urlInput}
                onkeydown={saveUrl}
                onblur={() => {
                    editingUrl = false;
                }}
                class="border-2 border-slate-900 text-slate-900 text-sm px-2 py-0.5 rounded font-mono w-72 outline-none"
            />
            <button
                onclick={() => saveUrl()}
                class="text-xs font-bold bg-slate-900 text-white px-2 py-1 rounded"
            >
                Save
            </button>
        {:else}
            <button
                onclick={startEditUrl}
                class="text-sm text-slate-400 hover:text-slate-700 font-mono truncate transition-colors text-left"
                title="Click to edit WebSocket URL"
            >
                {$wsUrl}
            </button>
        {/if}
    </div>
</header>
