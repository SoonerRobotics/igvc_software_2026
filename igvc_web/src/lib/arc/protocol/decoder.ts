import { ByteBuffer } from "flatbuffers";
import { type MessageWrapper } from "./types";
import { ArcCapability } from "../messages/messages/arc/arc-capability";
import { VectornavReport } from "../messages/messages";

class decoder {
    capability_ack(wrapper: MessageWrapper): ArcCapability {
        return ArcCapability.getRootAsArcCapability(new ByteBuffer(wrapper.payload));
    }

    vectornav(wrapper: MessageWrapper): VectornavReport {
        return VectornavReport.getRootAsVectornavReport(new ByteBuffer(wrapper.payload));
    }
}

export default new decoder();