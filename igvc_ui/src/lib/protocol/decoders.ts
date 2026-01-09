import { ByteBuffer } from "flatbuffers";
import { ImageFrame } from "../messages/messages/image-frame";
import { MessageWrapper } from "../types";
import { ArcCapability } from "../messages/messages/arc/arc-capability";

class decoder {
    image_frame(wrapper: MessageWrapper): ImageFrame {
        return ImageFrame.getRootAsImageFrame(new ByteBuffer(wrapper.payload));
    }

    capability_ack(wrapper: MessageWrapper): ArcCapability {
        return ArcCapability.getRootAsArcCapability(new ByteBuffer(wrapper.payload));
    }
}

export default new decoder();