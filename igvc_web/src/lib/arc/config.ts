// lib/arc/config.ts

export type ConfigSnapshot = {
    keys: Record<string, unknown>;
    presets: string[];
    currentPreset: string;
};

export type ConfigKeyChanged = {
    path: string;
    value: unknown;
};

export type ConfigPresetList = {
    presets: string[];
    currentPreset: string;
};

export type ConfigAck = {
    success: boolean;
    message?: string;
};

const dec = new TextDecoder();
const enc = new TextEncoder();

function decodeJson<T>(payload: Uint8Array): T {
    return JSON.parse(dec.decode(payload)) as T;
}

export const parseConfigSnapshot = (p: Uint8Array) => decodeJson<ConfigSnapshot>(p);
export const parseConfigKeyChanged = (p: Uint8Array) => decodeJson<ConfigKeyChanged>(p);
export const parseConfigPresetList = (p: Uint8Array) => decodeJson<ConfigPresetList>(p);
export const parseConfigAck = (p: Uint8Array) => decodeJson<ConfigAck>(p);

// ── Outgoing command payloads ─────────────────────────────────────────────────
//
// SetConfigKey wire format:
//   [2 bytes: uint16 LE path length][path UTF-8 bytes][value JSON UTF-8 bytes]
//
// Using a length-prefix instead of a null separator avoids the FlatBuffers
// null-termination issue where \0 in the byte array truncates string reads.

export function encodeSetConfigKey(path: string, value: unknown): Uint8Array {
    const pathBytes = enc.encode(path);
    const valueBytes = enc.encode(JSON.stringify(value));
    const out = new Uint8Array(2 + pathBytes.length + valueBytes.length);
    // uint16 LE path length
    out[0] = pathBytes.length & 0xff;
    out[1] = (pathBytes.length >> 8) & 0xff;
    out.set(pathBytes, 2);
    out.set(valueBytes, 2 + pathBytes.length);
    return out;
}

export const encodePresetName = (filename: string): Uint8Array => enc.encode(filename);