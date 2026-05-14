import { ByteBuffer } from "flatbuffers";

export enum Endianness {
    Little = "little",
    Big = "big"
}

export enum MessageType {
    // 2026
    ImageFrame = 0x01,
    ArcLog = 0x02,
    Gps = 0x03,
    Metric = 0x04,
    MetricHistory = 0x05,

    // Reserved Stuff
    CapabilityReq = 60_000,
    CapabilityAck = 60_001,
    CommandReq = 60_002,
    CommandAck = 60_003,
}

export interface MessageWrapper {
    type: MessageType;
    payload: Uint8Array;
}

export function makeMessageWrapper(type: MessageType, payload: Uint8Array): MessageWrapper {
    return {
        type,
        payload
    };
}

export function makeMessageWrapperBB(type: MessageType, payload: ByteBuffer): MessageWrapper {
    return {
        type,
        payload: payload.bytes()
    };
}