import { Builder, ByteBuffer } from "flatbuffers";
import { Purpose } from "./capabilities";
import { ArcCapability } from "../messages/messages/arc/arc-capability";

export function constructCapabilityRequest(visionMask: number, telemetryMask: number, miscMask: number): ArcCapability {
    const builder = new Builder(1024);
    const capabilityOffset = ArcCapability.createArcCapability(
        builder,
        BigInt(new Date().getTime()),
        0,
        Purpose.Req,
        visionMask,
        telemetryMask,
        miscMask
    );
    builder.finish(capabilityOffset);

    return ArcCapability.getRootAsArcCapability(new ByteBuffer(builder.asUint8Array()));
}