import { ByteBuffer } from "flatbuffers";
import { ImageFrame } from "./flatbuffers/flatbuffers/image-frame";
import { ArcLog } from "./flatbuffers/flatbuffers/arc/arc-log";

export const FLATBUFFER_USE_BIG_ENDIAN = false;

/**
 * FlatBufferType enum
 */
export enum FlatBufferType {
    IMAGE_FRAME = 0x01,
    ARC_LOG = 0x02,
}

/**
 * Helper to convert byte → enum
 */
export function flatBufferTypeFromByte(b: number): FlatBufferType | null {
    switch (b) {
        case FlatBufferType.IMAGE_FRAME:
            return FlatBufferType.IMAGE_FRAME;
        case FlatBufferType.ARC_LOG:
            return FlatBufferType.ARC_LOG;
        default:
            return null;
    }
}

/**
 * FlatBufferWrapper equivalent
 */
export class FlatBufferWrapper {
    messageType: FlatBufferType;
    payloadLength: number;
    payload: Uint8Array;

    constructor(type: FlatBufferType, payload: Uint8Array) {
        this.messageType = type;
        this.payloadLength = payload.length;
        this.payload = payload;
    }

    /**
     * Parse from raw byte buffer (e.g. WebSocket frame)
     */
    static fromBuffer(buffer: Uint8Array): FlatBufferWrapper {
        let offset = 0;

        const messageTypeByte = buffer[offset++];
        const messageType = flatBufferTypeFromByte(messageTypeByte);
        if (messageType === null) {
            throw new Error("Unknown FlatBufferType: " + messageTypeByte);
        }

        // Read payload length (BIG-ENDIAN, matches DataInputStream.readInt)
        let payloadLength = 0;
        if (FLATBUFFER_USE_BIG_ENDIAN) {
            payloadLength |= buffer[offset++] << 24;
            payloadLength |= buffer[offset++] << 16;
            payloadLength |= buffer[offset++] << 8;
            payloadLength |= buffer[offset++];
        } else {
            payloadLength |= buffer[offset++];
            payloadLength |= buffer[offset++] << 8;
            payloadLength |= buffer[offset++] << 16;
            payloadLength |= buffer[offset++] << 24;
        }

        if (payloadLength < 0 || offset + payloadLength > buffer.length) {
            throw new Error("Invalid payload length");
        }

        const payload = buffer.slice(offset, offset + payloadLength);

        return new FlatBufferWrapper(messageType, payload);
    }

    /**
     * Create wrapper from payload
     */
    static create(type: FlatBufferType, payload: Uint8Array): FlatBufferWrapper {
        return new FlatBufferWrapper(type, payload);
    }

    /**
     * Convert to wire format
     * | type (1) | length (4) | payload |
     */
    toByteArray(bigEndian = true): Uint8Array {
        const totalLength = 1 + 4 + this.payloadLength;
        const buffer = new Uint8Array(totalLength);

        let offset = 0;
        buffer[offset++] = this.messageType;

        if (bigEndian) {
            buffer[offset++] = (this.payloadLength >>> 24) & 0xff;
            buffer[offset++] = (this.payloadLength >>> 16) & 0xff;
            buffer[offset++] = (this.payloadLength >>> 8) & 0xff;
            buffer[offset++] = this.payloadLength & 0xff;
        } else {
            buffer[offset++] = this.payloadLength & 0xff;
            buffer[offset++] = (this.payloadLength >>> 8) & 0xff;
            buffer[offset++] = (this.payloadLength >>> 16) & 0xff;
            buffer[offset++] = (this.payloadLength >>> 24) & 0xff;
        }

        buffer.set(this.payload, offset);
        return buffer;
    }
}

/**
 * FlatBufferConverter equivalent
 */
export class FlatBufferConverter {
    private static checkType(
        wrapper: FlatBufferWrapper,
        expected: FlatBufferType
    ) {
        if (wrapper.messageType !== expected) {
            throw new Error(
                `FlatBufferWrapper is not of type ${FlatBufferType[expected]}`
            );
        }
    }

    static asImageFrame(wrapper: FlatBufferWrapper): ImageFrame {
        this.checkType(wrapper, FlatBufferType.IMAGE_FRAME);
        return ImageFrame.getRootAsImageFrame(
            new ByteBuffer(wrapper.payload)
        );
    }

    static asArcLog(wrapper: FlatBufferWrapper): ArcLog {
        this.checkType(wrapper, FlatBufferType.ARC_LOG);
        return ArcLog.getRootAsArcLog(
            new ByteBuffer(wrapper.payload)
        );
    }
}
