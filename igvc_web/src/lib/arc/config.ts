// lib/arc/config.ts
// Parses the JSON payloads inside ArcData wrappers for config identifiers.
// The C# side encodes all config payloads as UTF-8 JSON in the dataPayload field.

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

function decodeJson<T>(payload: Uint8Array): T {
    return JSON.parse(new TextDecoder().decode(payload)) as T;
}

export function parseConfigSnapshot(payload: Uint8Array): ConfigSnapshot {
    return decodeJson<ConfigSnapshot>(payload);
}

export function parseConfigKeyChanged(payload: Uint8Array): ConfigKeyChanged {
    return decodeJson<ConfigKeyChanged>(payload);
}

export function parseConfigPresetList(payload: Uint8Array): ConfigPresetList {
    return decodeJson<ConfigPresetList>(payload);
}

export function parseConfigAck(payload: Uint8Array): ConfigAck {
    return decodeJson<ConfigAck>(payload);
}

// ── Outgoing command payloads ────────────────────────────────────────────────
// These encode the UTF-8 byte payloads that ArcConfigHandler expects.

const enc = new TextEncoder();

/** SetConfigKey payload: "{path}\0{jsonValue}" */
export function encodeSetConfigKey(path: string, value: unknown): Uint8Array {
    return enc.encode(`${path}\0${JSON.stringify(value)}`);
}

/** LoadPreset payload: filename string */
export function encodeLoadPreset(filename: string): Uint8Array {
    return enc.encode(filename);
}

/** SavePreset payload: filename string */
export function encodeSavePreset(filename: string): Uint8Array {
    return enc.encode(filename);
}