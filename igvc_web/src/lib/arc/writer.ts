import { MessageType } from "./type";

const MAGIC_SIZE = 4;
const TYPE_SIZE = 2;
const LENGTH_SIZE = 4;
const FLAGS_SIZE = 2;
const HEADER_SIZE = MAGIC_SIZE + TYPE_SIZE + LENGTH_SIZE + FLAGS_SIZE;
const CRC_SIZE = 4;
const FLAG_CRC: number = 0x0001;

const NETWORKING_MAGIC = new Uint8Array([0x49, 0x47, 0x56, 0x43]); // "IGVC"

function crc32(data: Uint8Array): number {
    const table = crc32Table();
    let crc = 0xffffffff;
    for (let i = 0; i < data.length; i++) {
        crc = (crc >>> 8) ^ table[(crc ^ data[i]) & 0xff];
    }
    return (crc ^ 0xffffffff) >>> 0;
}

let _crc32Table: Uint32Array | null = null;
function crc32Table(): Uint32Array {
    if (_crc32Table) return _crc32Table;
    _crc32Table = new Uint32Array(256);
    for (let i = 0; i < 256; i++) {
        let c = i;
        for (let j = 0; j < 8; j++) {
            c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
        }
        _crc32Table[i] = c;
    }
    return _crc32Table;
}

export type Endianness = "little" | "big";

export class MessageWriter {
    static write(
        type: MessageType,
        payload: Uint8Array,
        endianness: Endianness,
        includeCrc = true
    ): Uint8Array {
        if (payload.length <= 0) {
            throw new RangeError(`Payload length ${payload.length} is invalid`);
        }

        const crcSize = includeCrc ? CRC_SIZE : 0;
        const frameSize = HEADER_SIZE + payload.length + crcSize;
        const buffer = new Uint8Array(frameSize);

        MessageWriter._writeHeader(buffer, type, payload.length, endianness, includeCrc);
        buffer.set(payload, HEADER_SIZE);

        if (includeCrc) {
            const crc = crc32(buffer.subarray(0, HEADER_SIZE + payload.length));
            const view = new DataView(buffer.buffer);
            const offset = HEADER_SIZE + payload.length;
            if (endianness === "little") {
                view.setUint32(offset, crc, true);
            } else {
                view.setUint32(offset, crc, false);
            }
        }

        return buffer;
    }

    private static _writeHeader(
        buffer: Uint8Array,
        type: MessageType,
        payloadLength: number,
        endianness: Endianness,
        includeCrc: boolean
    ): void {
        buffer.set(NETWORKING_MAGIC, 0);

        const view = new DataView(buffer.buffer);
        const le = endianness === "little";

        view.setUint16(MAGIC_SIZE, type, le);
        view.setInt32(MAGIC_SIZE + TYPE_SIZE, payloadLength, le);
        view.setUint16(MAGIC_SIZE + TYPE_SIZE + LENGTH_SIZE, includeCrc ? FLAG_CRC : 0, le);
    }
}