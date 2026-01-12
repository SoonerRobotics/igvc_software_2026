<script lang="ts">
    import { subscribe } from "$lib/arc/socket";
    import { MessageType } from "$lib/arc/protocol/types";
    import decoders from "$lib/arc/protocol/decoders";
    import type { MetricSample } from "$lib/arc/messages/messages/performance/metric-sample";
    import MetricGraph from "../../components/metric-graph.svelte";

    let plotRef: MetricGraph;
    let yoloRef: MetricGraph;

    subscribe<MetricSample>(
        MessageType.Metric,
        decoders.metric_sample,
        (msg) => {
            if (msg.group() === "vision" && msg.name() === "Processing Time") {
                plotRef.addSample(new Number(msg.timestampUnixMs()).valueOf(), msg.value());
            }

            if (msg.group() === "yolo" && msg.name() === "Detection Time") {
                yoloRef.addSample(new Number(msg.timestampUnixMs()).valueOf(), msg.value());
            }
        },
    );
</script>

<MetricGraph
    bind:this={plotRef}
    title="Vision Processing Time"
    unit="ms"
    timeWindowMs={30_000}
/>

<MetricGraph
    bind:this={yoloRef}
    title="Yolo Detection Time"
    unit="ms"
    timeWindowMs={30_000}
/>
