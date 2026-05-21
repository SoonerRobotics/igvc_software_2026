import { MessageType } from "./type";
import { MessageWrapper } from "./wrapper";

const MAGIC_SIZE = 4;
const TYPE_SIZE = 2;
const LENGTH_SIZE = 4;
const FLAGS_SIZE = 2;
const HEADER_SIZE = MAGIC_SIZE + TYPE_SIZE + LENGTH_SIZE + FLAGS_SIZE;
const CRC_SIZE = 4;
const FLAG_CRC = 0x0001;

const NETWORKING_MAGIC = new Uint8Array([0x49, 0x47, 0x56, 0x43]); // "IGVC"

export type Endianness = "little" | "big";

function crc32(data: Uint8Array): number {
    const table = getCrc32Table();
    let crc = 0xffffffff;
    for (let i = 0; i < data.length; i++) {
        crc = (crc >>> 8) ^ table[(crc ^ data[i]) & 0xff];
    }
    return (crc ^ 0xffffffff) >>> 0;
}

let _table: Uint32Array | null = null;
function getCrc32Table(): Uint32Array {
    if (_table) return _table;
    _table = new Uint32Array(256);
    for (let i = 0; i < 256; i++) {
        let c = i;
        for (let j = 0; j < 8; j++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
        _table[i] = c;
    }
    return _table;
}

export class MessageAccumulator {
    private readonly _endianness: Endianness;
    private readonly _onMessage: (msg: MessageWrapper) => void;
    private readonly _maxMessageLength: number;
    private readonly _maxBufferSize: number;

    private _buffer: Uint8Array;
    private _writePos = 0;

    constructor(
        endianness: Endianness,
        onMessage: (msg: MessageWrapper) => void,
        options: {
            initialCapacity?: number;
            maxMessageLength?: number;
            maxBufferSize?: number;
        } = {}
    ) {
        this._endianness = endianness;
        this._onMessage = onMessage;
        this._maxMessageLength = options.maxMessageLength ?? 2 * 1024 * 1024;
        this._maxBufferSize = options.maxBufferSize ?? 4 * 1024 * 1024;
        this._buffer = new Uint8Array(options.initialCapacity ?? 4096);
    }

    append(bytes: Uint8Array): void {
        const required = this._writePos + bytes.length;
        if (required > this._maxBufferSize) {
            throw new Error(
                `MessageAccumulator buffer overflow: ${required} > ${this._maxBufferSize}`
            );
        }
        this._ensureCapacity(required);
        this._buffer.set(bytes, this._writePos);
        this._writePos += bytes.length;
        this._process();
    }

    private _process(): void {
        const le = this._endianness === "little";
        let readPos = 0;

        while (this._writePos - readPos >= HEADER_SIZE) {
            const available = this._writePos - readPos;
            const view = new DataView(this._buffer.buffer, this._buffer.byteOffset + readPos, available);

            if (!this._matchesMagic(readPos)) {
                readPos++;
                continue;
            }

            const type = view.getUint16(MAGIC_SIZE, le);
            const length = view.getInt32(MAGIC_SIZE + TYPE_SIZE, le);

            if (length < 0 || length > this._maxMessageLength) {
                throw new Error(`Invalid message length ${length}`);
            }

            const flags = view.getUint16(MAGIC_SIZE + TYPE_SIZE + LENGTH_SIZE, le);
            const hasCrc = (flags & FLAG_CRC) !== 0;
            const crcSize = hasCrc ? CRC_SIZE : 0;
            const frameSize = HEADER_SIZE + length + crcSize;

            if (available < frameSize) break;

            if (hasCrc) {
                const headerAndPayload = this._buffer.subarray(readPos, readPos + HEADER_SIZE + length);
                const computed = crc32(headerAndPayload);
                const received = view.getUint32(HEADER_SIZE + length, le);
                if (computed !== received) {
                    throw new Error(
                        `CRC mismatch: computed 0x${computed.toString(16).toUpperCase().padStart(8, "0")}, ` +
                        `received 0x${received.toString(16).toUpperCase().padStart(8, "0")}`
                    );
                }
            }

            const payload = this._buffer.slice(readPos + HEADER_SIZE, readPos + HEADER_SIZE + length);
            this._onMessage(MessageWrapper.fromPooled(type as MessageType, payload, length));

            readPos += frameSize;
        }

        if (readPos > 0) {
            const remaining = this._writePos - readPos;
            if (remaining > 0) {
                this._buffer.copyWithin(0, readPos, this._writePos);
            }
            this._writePos = remaining;
        }
    }

    private _matchesMagic(offset: number): boolean {
        for (let i = 0; i < MAGIC_SIZE; i++) {
            if (this._buffer[offset + i] !== NETWORKING_MAGIC[i]) return false;
        }
        return true;
    }

    private _ensureCapacity(required: number): void {
        if (this._buffer.length >= required) return;
        const newSize = Math.max(this._buffer.length * 2, required);
        const newBuffer = new Uint8Array(newSize);
        newBuffer.set(this._buffer.subarray(0, this._writePos));
        this._buffer = newBuffer;
    }

    reset(): void {
        this._writePos = 0;
    }
}