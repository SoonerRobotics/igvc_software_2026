// lib/arc/command.ts
// Builds a CommandReq MessageWrapper to send to the robot.
//
// Wire format (little-endian, matches C# ArcCommand FlatBuffer + MessageWriter):
//   The payload written into the ArcCommand FlatBuffer table:
//     commandId : uint16  (ArcCommandId)
//     payload   : [ubyte] (arbitrary bytes, command-specific)
//
// We build the FlatBuffer manually here to avoid needing a generated
// ArcCommand builder — the table is simple enough to inline.
// If you have a generated ArcCommand builder, swap this body out for it.

import { Builder, ByteBuffer } from "flatbuffers";
import { MessageType } from "./type";
import { MessageWrapper } from "./wrapper";
import { ArcCommandId } from "../messages/messages/arc/arc-command-id";

/**
 * Builds a binary MessageWrapper of type CommandReq containing an ArcCommand
 * FlatBuffer with the given commandId and payload bytes.
 */
export function buildCommandReq(
    commandId: ArcCommandId,
    payload: Uint8Array = new Uint8Array(0)
): MessageWrapper {
    const builder = new Builder(64 + payload.byteLength);

    // Create the payload vector
    const payloadOffset = builder.createByteVector(payload);

    // ArcCommand table layout (must match your .fbs schema field order):
    //   field 0: command_id : uint16
    //   field 1: payload    : [ubyte]
    //
    // FlatBuffers tables are built in reverse field order.
    builder.startObject(2);
    builder.addFieldInt16(0, commandId, ArcCommandId.UnknownCommand);
    builder.addFieldOffset(1, payloadOffset, 0);
    const root = builder.endObject();
    builder.finish(root);

    const bytes = builder.asUint8Array();
    return MessageWrapper.from(MessageType.CommandReq, bytes);
}