<script lang="ts">
    import CameraFeed from "../../components/camera.svelte";
    import SwerveViz from "../../components/swerve.svelte";
    import {
        latestImageFrame,
        vectornav,
        isConnected,
    } from "$lib/arc/connection";

    let swerveWheels = $state([
        { angle: 0, speed: 0 },
        { angle: 0, speed: 0 },
        { angle: 0, speed: 0 },
        { angle: 0, speed: 0 },
    ]);

    function headingLabel(deg: number): string {
        const dirs = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];
        return dirs[Math.round(deg / 45) % 8];
    }

    function fmt(n: number | undefined | null, decimals = 2): string {
        if (n == null) return "—";
        return n.toFixed(decimals);
    }

    const heading = $derived(
        $vectornav ? (($vectornav.yaw * 180) / Math.PI + 360) % 360 : null,
    );
</script>

<div class="h-full w-full p-4 overflow-auto">
    <div class="max-w-7xl mx-auto flex flex-col gap-4 h-full">
        <div class="flex items-center justify-between shrink-0">
            <h1 class="text-xl font-black tracking-tight text-slate-900">
                Dashboard
            </h1>
            <div class="flex items-center gap-2">
                <div
                    class="w-2 h-2 rounded-full {$isConnected
                        ? 'bg-emerald-500'
                        : 'bg-slate-300'}"
                ></div>
                <span class="text-sm text-slate-500 font-medium"
                    >{$isConnected ? "Live" : "No data"}</span
                >
            </div>
        </div>

        <div class="grid grid-cols-12 gap-4 flex-1 min-h-0">
            <div class="col-span-12 lg:col-span-8 flex flex-col gap-4">
                <CameraFeed label="Forward" frame={$latestImageFrame} />

                <CameraFeed label="Depth" frame={null} />
            </div>

            <div class="col-span-12 lg:col-span-4 flex flex-col gap-4">
                <div class="bg-white rounded-xl border-2 border-slate-200 p-4">
                    <span
                        class="text-[10px] font-bold tracking-widest uppercase text-slate-400"
                        >Heading</span
                    >

                    <div class="flex items-center justify-between mt-3">
                        <div class="relative w-20 h-20">
                            <svg viewBox="0 0 80 80" class="w-full h-full">
                                <circle
                                    cx="40"
                                    cy="40"
                                    r="36"
                                    fill="none"
                                    stroke="#e2e8f0"
                                    stroke-width="2"
                                />

                                <text
                                    x="40"
                                    y="10"
                                    text-anchor="middle"
                                    font-size="7"
                                    font-family="monospace"
                                    font-weight="bold"
                                    fill="#94a3b8">N</text
                                >
                                <text
                                    x="40"
                                    y="75"
                                    text-anchor="middle"
                                    font-size="7"
                                    font-family="monospace"
                                    font-weight="bold"
                                    fill="#94a3b8">S</text
                                >
                                <text
                                    x="8"
                                    y="43"
                                    text-anchor="middle"
                                    font-size="7"
                                    font-family="monospace"
                                    font-weight="bold"
                                    fill="#94a3b8">W</text
                                >
                                <text
                                    x="72"
                                    y="43"
                                    text-anchor="middle"
                                    font-size="7"
                                    font-family="monospace"
                                    font-weight="bold"
                                    fill="#94a3b8">E</text
                                >

                                <g transform="rotate({heading ?? 0}, 40, 40)">
                                    <polygon
                                        points="40,10 37,40 43,40"
                                        fill="#ef4444"
                                    />
                                    <polygon
                                        points="40,70 37,40 43,40"
                                        fill="#cbd5e1"
                                    />
                                </g>

                                <circle cx="40" cy="40" r="3" fill="#1e293b" />
                            </svg>
                        </div>

                        <div class="flex flex-col items-end">
                            <span
                                class="text-4xl font-black tabular-nums text-slate-900 leading-none"
                            >
                                {fmt(heading, 1)}°
                            </span>
                            <span class="text-lg font-bold text-slate-400 mt-1">
                                {heading != null ? headingLabel(heading) : "—"}
                            </span>
                        </div>
                    </div>
                </div>

                <div class="bg-white rounded-xl border-2 border-slate-200 p-4">
                    <span
                        class="text-[10px] font-bold tracking-widest uppercase text-slate-400"
                        >GPS</span
                    >
                    <div class="flex flex-col gap-2 mt-3">
                        <div class="flex justify-between items-baseline">
                            <span class="text-xs font-semibold text-slate-500"
                                >Latitude</span
                            >
                            <span
                                class="text-sm font-black font-mono text-slate-900"
                                >{fmt($vectornav?.lat, 6)}°</span
                            >
                        </div>
                        <div class="w-full h-px bg-slate-100"></div>
                        <div class="flex justify-between items-baseline">
                            <span class="text-xs font-semibold text-slate-500"
                                >Longitude</span
                            >
                            <span
                                class="text-sm font-black font-mono text-slate-900"
                                >{fmt($vectornav?.lng, 6)}°</span
                            >
                        </div>
                        <div class="w-full h-px bg-slate-100"></div>
                    </div>
                </div>

                <div class="bg-white rounded-xl border-2 border-slate-200 p-4">
                    <span
                        class="text-[10px] font-bold tracking-widest uppercase text-slate-400"
                        >IMU</span
                    >
                    <div class="flex flex-col gap-2 mt-3">
                        {#each [{ label: "Roll", value: $vectornav?.roll }, { label: "Pitch", value: $vectornav?.pitch }, { label: "Yaw", value: $vectornav?.yaw }] as row}
                            <div class="flex justify-between items-baseline">
                                <span
                                    class="text-xs font-semibold text-slate-500"
                                    >{row.label}</span
                                >
                                <span
                                    class="text-sm font-black font-mono text-slate-900"
                                >
                                    {row.value != null
                                        ? fmt((row.value * 180) / Math.PI, 1) +
                                          "°"
                                        : "—"}
                                </span>
                            </div>
                            <div
                                class="w-full h-px bg-slate-100 last:hidden"
                            ></div>
                        {/each}
                    </div>
                </div>

                <SwerveViz wheels={swerveWheels} />
            </div>
        </div>
    </div>
</div>
