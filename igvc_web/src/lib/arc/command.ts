// lib/arc/command.ts
import { Builder } from "flatbuffers";
import { ArcCommand } from "../messages/messages/arc/arc-command";
import { MessageType } from "./type";
import { MessageWriter } from "./writer";
import { ArcCommandPurpose } from "../messages/messages/arc/arc-command-purpose";
import { ArcCommandId } from "../messages/messages/arc/arc-command-id";

let _sequenceNumber = 0;

/**
 * Builds a fully-framed byte array ready to pass directly to ws.send().
 * Frame: [IGVC magic][type][length][flags][FlatBuffer payload][CRC32]
 */
export function buildCommandReq(
    commandId: ArcCommandId,
    data: Uint8Array = new Uint8Array(0)
): Uint8Array {
    const builder = new Builder(128 + data.byteLength);
    const dataVector = ArcCommand.createDataVector(builder, data);
    const root = ArcCommand.createArcCommand(
        builder,
        BigInt(Date.now() * 1000),  // timestamp in microseconds
        ++_sequenceNumber,
        ArcCommandPurpose.Request,
        commandId,
        dataVector
    );
    builder.finish(root);

    return MessageWriter.write(MessageType.CommandReq, builder.asUint8Array(), "little", true);
}