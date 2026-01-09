import { Endianness, MessageType, MessageWrapper, makeMessageWrapper } from "../types";
import { crc32 } from "./crc32";

const Magic = new Uint8Array([0x49, 0x47, 0x56, 0x43]);

const MagicSize = 4;
const HeaderSize = MagicSize + 6;
const CrcSize = 4;

export class MessageAccumulator {
    private buffer: Uint8Array;
    private length = 0;

    constructor(
        private readonly endianness: Endianness,
        private readonly onMessage: (msg: MessageWrapper) => void,
        initialCapacity = 4096,
        private readonly maxMessageLength = 512 * 1024,
        private readonly maxBufferSize = 1024 * 1024
    ) {
        this.buffer = new Uint8Array(initialCapacity);
    }

    append(bytes: Uint8Array) {
        if (this.length + bytes.length > this.maxBufferSize) {
            throw new Error("MessageAccumulator buffer overflow");
        }

        this.ensureCapacity(this.length + bytes.length);
        this.buffer.set(bytes, this.length);
        this.length += bytes.length;

        this.process();
    }

    private ensureCapacity(size: number) {
        if (this.buffer.length >= size) return;

        const next = new Uint8Array(Math.max(size, this.buffer.length * 2));
        next.set(this.buffer.subarray(0, this.length));
        this.buffer = next;
    }

    private process() {
        let offset = 0;
        const view = new DataView(this.buffer.buffer);

        while (this.length - offset >= HeaderSize) {
            // Check magic
            if (!this.matchesMagic(offset)) {
                offset += 1;
                continue;
            }

            const little = this.endianness === Endianness.Little;
            const type = view.getUint16(offset + MagicSize, little);
            const payloadLength = view.getInt32(offset + MagicSize + 2, little);

            if (payloadLength < 0 || payloadLength >= this.maxMessageLength) {
                throw new Error(`Invalid message length ${payloadLength}`);
            }

            const frameSize = HeaderSize + payloadLength + CrcSize;
            if (this.length - offset < frameSize) break;

            const crcStart = offset + HeaderSize + payloadLength;
            const receivedCrc = view.getUint32(crcStart, true);

            const crcData = this.buffer.subarray(offset, offset + HeaderSize + payloadLength);
            const computedCrc = crc32(crcData);

            if (computedCrc !== receivedCrc) {
                throw new Error("CRC mismatch in message");
            }

            const payload = this.buffer.slice(
                offset + HeaderSize,
                offset + HeaderSize + payloadLength
            );

            this.onMessage(makeMessageWrapper(type as MessageType, payload));

            offset += frameSize;
        }

        // Compact buffer
        if (offset > 0) {
            this.buffer.copyWithin(0, offset, this.length);
            this.length -= offset;
        }
    }

    private matchesMagic(offset: number): boolean {
        for (let i = 0; i < MagicSize; i++) {
            if (this.buffer[offset + i] !== Magic[i]) return false;
        }
        return true;
    }
}
