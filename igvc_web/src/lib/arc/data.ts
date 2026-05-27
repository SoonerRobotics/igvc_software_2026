import { ByteBuffer } from "flatbuffers";
import { decodeJson } from "./encoders";

export interface ArcLog {
    cat: string;
    level: number;
    timestamp: number;
    id: number;
    name: string;
    message: string;
}

export function buildArcData_Log(msg: Uint8Array): ArcLog {
    const view = new DataView(msg.buffer, msg.byteOffset, msg.byteLength);
    let offset = 0;

    function readString(): string {
        let len = 0;
        let shift = 0;
        while (true) {
            const b = view.getUint8(offset++);
            len |= (b & 0x7f) << shift;
            if ((b & 0x80) === 0) break;
            shift += 7;
        }
        const bytes = new Uint8Array(view.buffer, view.byteOffset + offset, len);
        offset += len;
        return new TextDecoder("utf-8").decode(bytes);
    }

    const cat = readString();
    const level = view.getUint8(offset); offset += 1;
    const id = view.getInt32(offset, true); offset += 4;
    const name = readString();
    const message = readString();

    return { cat, level, id, name, message, timestamp: Date.now() };
}

export function buildArcData_RobotState(msg: Uint8Array) {
    const json = decodeJson<{
        Estopped: boolean;
        IsSimulation: boolean;
        Mission: number;
        Mode: number;
        MotionAllowed: boolean;
    }>(msg);
    return json;
}

export interface PropertyChanged {
    subsystem: string,
    property: string,
    value: string,
}

export function buildArcData_PropertyChanged(msg: Uint8Array): PropertyChanged {
    const view = new DataView(msg.buffer, msg.byteOffset, msg.byteLength);
    let offset = 0;

    function readString(): string {
        let len = 0;
        let shift = 0;
        while (true) {
            const b = view.getUint8(offset++);
            len |= (b & 0x7f) << shift;
            if ((b & 0x80) === 0) break;
            shift += 7;
        }
        const bytes = new Uint8Array(view.buffer, view.byteOffset + offset, len);
        offset += len;
        return new TextDecoder("utf-8").decode(bytes);
    }

    const subsystem = readString();
    const property = readString();
    const value = readString();

    return { subsystem, property, value };
}