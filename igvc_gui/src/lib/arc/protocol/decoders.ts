import { ByteBuffer } from "flatbuffers";
import { type MessageWrapper } from "./types";
import { ImageFrame } from "../messages/messages/image-frame";
import { ArcCapability } from "../messages/messages/arc/arc-capability";
import { VectornavReport } from "../messages/messages";
import { MetricHistory } from "../messages/messages/performance/metric-history";
import { MetricSample } from "../messages/messages/performance/metric-sample";

class decoder {
    image_frame(wrapper: MessageWrapper): ImageFrame {
        return ImageFrame.getRootAsImageFrame(new ByteBuffer(wrapper.payload));
    }

    capability_ack(wrapper: MessageWrapper): ArcCapability {
        return ArcCapability.getRootAsArcCapability(new ByteBuffer(wrapper.payload));
    }

    vectornav(wrapper: MessageWrapper): VectornavReport {
        return VectornavReport.getRootAsVectornavReport(new ByteBuffer(wrapper.payload));
    }

    metric_history(wrapper: MessageWrapper): MetricHistory {
        return MetricHistory.getRootAsMetricHistory(new ByteBuffer(wrapper.payload));
    }

    metric_sample(wrapper: MessageWrapper): MetricSample {
        return MetricSample.getRootAsMetricSample(new ByteBuffer(wrapper.payload));
    }
}

export default new decoder();