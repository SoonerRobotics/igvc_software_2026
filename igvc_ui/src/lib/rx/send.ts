import { socket$ } from "./socket";
import { MessageWriter } from "../protocol/message_writer";
import { Endianness, MessageType } from "../types";

export function sendMessage(type: MessageType, payload: Uint8Array) {
    const frame = MessageWriter.write(type, payload, Endianness.Little);
    socket$.next(frame.buffer as ArrayBuffer);
}
