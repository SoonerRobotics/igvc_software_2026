<script lang="ts">
    import { onMount, onDestroy } from "svelte";
    import uPlot from "uplot";
    import "uplot/dist/uPlot.min.css";

    type Sample = {
        tsMs: number;
        value: number;
    };

    export let title = "";
    export let unit = "";
    export let timeWindowMs = 30_000;
    export let width = 600;
    export let height = 200;
    export let stroke = "#4ade80";

    let container: HTMLDivElement;
    let plot: uPlot | null = null;

    let samples: Sample[] = [];

    export function addSample(tsMs: number, value: number) {
        samples.push({ tsMs, value });

        const cutoff = tsMs - timeWindowMs;
        while (samples.length && samples[0].tsMs < cutoff) {
            samples.shift();
        }

        updatePlot();
    }

    export function clear() {
        samples = [];
        plot?.setData([[], []]);
    }

    function updatePlot() {
        if (!plot || samples.length < 2) return;

        const xs = new Array<number>(samples.length);
        const ys = new Array<number>(samples.length);

        for (let i = 0; i < samples.length; i++) {
            xs[i] = samples[i].tsMs / 1000;
            ys[i] = samples[i].value;
        }

        plot.setData([xs, ys]);
    }

    onMount(() => {
        plot = new uPlot(
            {
                width,
                height,
                title,
                scales: {
                    x: { time: true },
                },
                axes: [
                    { stroke: "#9ca3af" },
                    {
                        stroke: "#9ca3af",
                        label: unit,
                    },
                ],
                series: [
                    {},
                    {
                        label: title,
                        stroke,
                        width: 2,
                    },
                ],
            },
            [[], []],
            container,
        );
    });

    onDestroy(() => {
        plot?.destroy();
        plot = null;
    });
</script>

<div class="p-4 bg-gray-300 rounded-lg">
    <div bind:this={container}></div>
</div>
