import { Builder, ByteBuffer } from "flatbuffers";
import { Purpose } from "./capabilities";
import { ArcCapability } from "../messages/messages/arc/arc-capability";
import { ArcCommand } from "../messages/messages/arc/arc-command";

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

export function constructCommandRequest(commandType: number, payload?: Uint8Array): ArcCommand {
    const builder = new Builder(1024);
    const dataOffset = builder.createByteVector(payload ? payload : new Uint8Array());
    const payloadOffset = ArcCommand.createArcCommand(
        builder,
        BigInt(new Date().getTime()),
        0,
        Purpose.Req,
        commandType,
        dataOffset
    );
    builder.finish(payloadOffset);

    return ArcCommand.getRootAsArcCommand(new ByteBuffer(builder.asUint8Array()));
}