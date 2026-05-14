<script lang="ts">
    const {
        wheels = [
            { angle: 0, speed: 0 },
            { angle: 0, speed: 0 },
            { angle: 0, speed: 0 },
            { angle: 0, speed: 0 },
        ],
    } = $props<{
        wheels?: { angle: number; speed: number }[];
    }>();

    const wheelLabels = ["FL", "FR", "BL", "BR"];
    const wheelPositions = [
        [38, 38], // FL
        [112, 38], // FR
        [38, 112], // BL
        [112, 112], // BR
    ];
</script>

<div class="flex flex-col gap-1">
    <span class="text-[10px] font-bold tracking-widest uppercase text-slate-400"
        >Swerve</span
    >
    <div class="bg-slate-100 rounded-xl border-2 border-slate-200 p-2">
        <svg
            viewBox="0 0 150 150"
            class="w-full aspect-square max-w-[180px] mx-auto"
        >
            <rect
                x="28"
                y="28"
                width="94"
                height="94"
                rx="8"
                fill="white"
                stroke="#cbd5e1"
                stroke-width="2"
            />

            <polygon points="75,30 70,40 80,40" fill="#1e293b" />
            <text
                x="75"
                y="20"
                text-anchor="middle"
                font-size="7"
                font-family="monospace"
                fill="#94a3b8"
                font-weight="bold">FWD</text
            >

            <line
                x1="75"
                y1="50"
                x2="75"
                y2="100"
                stroke="#e2e8f0"
                stroke-width="1"
            />
            <line
                x1="50"
                y1="75"
                x2="100"
                y2="75"
                stroke="#e2e8f0"
                stroke-width="1"
            />

            {#each wheels as wheel, i}
                {@const [cx, cy] = wheelPositions[i]}
                {@const speed = wheel.speed}
                {@const speedColor =
                    speed > 0.05
                        ? "#22c55e"
                        : speed < -0.05
                          ? "#ef4444"
                          : "#94a3b8"}

                <g transform="translate({cx},{cy}) rotate({wheel.angle})">
                    <rect
                        x="-10"
                        y="-5"
                        width="20"
                        height="10"
                        rx="2"
                        fill={speedColor}
                        fill-opacity="0.15"
                        stroke={speedColor}
                        stroke-width="1.5"
                    />

                    {#if Math.abs(speed) > 0.05}
                        {@const arrowLen = Math.abs(speed) * 8}
                        {@const dir = speed > 0 ? -1 : 1}
                        <line
                            x1="0"
                            y1="0"
                            x2="0"
                            y2={dir * arrowLen}
                            stroke={speedColor}
                            stroke-width="1.5"
                            marker-end="url(#arrow-{speed > 0 ? 'fwd' : 'rev'})"
                        />
                    {/if}

                    <circle cx="0" cy="0" r="1.5" fill={speedColor} />
                </g>

                <text
                    x={i % 2 === 0 ? cx - 16 : cx + 16}
                    y={cy + 3}
                    text-anchor={i % 2 === 0 ? "end" : "start"}
                    font-size="7"
                    font-family="monospace"
                    fill="#64748b"
                    font-weight="bold">{wheelLabels[i]}</text
                >
            {/each}

            <defs>
                <marker
                    id="arrow-fwd"
                    markerWidth="4"
                    markerHeight="4"
                    refX="2"
                    refY="2"
                    orient="auto"
                >
                    <path d="M0,0 L4,2 L0,4 Z" fill="#22c55e" />
                </marker>
                <marker
                    id="arrow-rev"
                    markerWidth="4"
                    markerHeight="4"
                    refX="2"
                    refY="2"
                    orient="auto"
                >
                    <path d="M0,0 L4,2 L0,4 Z" fill="#ef4444" />
                </marker>
            </defs>
        </svg>

        <div class="grid grid-cols-4 gap-1 mt-1">
            {#each wheels as wheel, i}
                <div class="flex flex-col items-center">
                    <span class="text-[9px] font-bold text-slate-400"
                        >{wheelLabels[i]}</span
                    >
                    <span class="text-[10px] font-mono font-bold text-slate-700"
                        >{wheel.angle.toFixed(0)}°</span
                    >
                </div>
            {/each}
        </div>
    </div>
</div>
