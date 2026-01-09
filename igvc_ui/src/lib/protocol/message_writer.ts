import { ByteBuffer } from "flatbuffers";
import { Endianness, MessageType, MessageWrapper } from "../types";
import { crc32 } from "./crc32";

const Magic = new Uint8Array([0x49, 0x47, 0x56, 0x43]);

const MagicSize = 4;
const HeaderSize = MagicSize + 6;
const CrcSize = 4;

export class MessageWriter {
    static write(
        type: MessageType,
        payload: Uint8Array,
        endianness: Endianness = Endianness.Little,
        maxMessageLength = 64 * 1024
    ): Uint8Array {
        if (payload.length <= 0 || payload.length > maxMessageLength) {
            throw new RangeError(`Payload length ${payload.length} is invalid`);
        }

        const frameSize = HeaderSize + payload.length + CrcSize;
        const buffer = new Uint8Array(frameSize);
        const view = new DataView(buffer.buffer);

        // Magic
        buffer.set(Magic, 0);

        // Type + Length
        const little = endianness === Endianness.Little;
        view.setUint16(MagicSize, type, little);
        view.setInt32(MagicSize + 2, payload.length, little);

        // Payload
        buffer.set(payload, HeaderSize);

        // CRC32C (type + length + payload)
        const crc = crc32(buffer.subarray(0, HeaderSize + payload.length));
        view.setUint32(HeaderSize + payload.length, crc, true);

        return buffer;
    }

    static writeByteBuffer(
        type: MessageType,
        payload: ByteBuffer,
        endianness: Endianness = Endianness.Little,
        maxMessageLength = 64 * 1024
    ): Uint8Array {
        return this.write(
            type,
            new Uint8Array(
                payload.bytes().buffer,
                payload.bytes().byteOffset,
                payload.bytes().length
            ),
            endianness,
            maxMessageLength
        );
    }

    static writeWrapper(
        type: MessageType,
        payload: MessageWrapper,
        endianness: Endianness = Endianness.Little,
        maxMessageLength = 64 * 1024
    ): Uint8Array {
        return this.writeByteBuffer(
            type,
            new ByteBuffer(payload.payload),
            endianness,
            maxMessageLength
        );
    }
}
