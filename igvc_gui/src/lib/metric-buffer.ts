import type { MetricSample } from "./arc/messages/messages/performance/metric-sample"

// metricBuffer.ts
export class MetricBuffer {
    private samples: MetricSample[] = []
    constructor(private maxMs: number) { }

    add(sample: MetricSample) {
        this.samples.push(sample)
        const cutoff = Number(sample.timestampUnixMs()).valueOf() - this.maxMs
        while (this.samples.length && Number(this.samples[0].timestampUnixMs()).valueOf() < cutoff) {
            this.samples.shift()
        }
    }

    get(): MetricSample[] {
        return this.samples
    }
}
