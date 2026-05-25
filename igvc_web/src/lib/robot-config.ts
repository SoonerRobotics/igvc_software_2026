// lib/robot-config.ts
// Drop-in config slice for useRobotStore.
// Import and spread into the store definition, then wire onMessage to call handleConfigMessage.

import {
    ConfigAck,
    ConfigSnapshot,
    parseConfigAck,
    parseConfigKeyChanged,
    parseConfigPresetList,
    parseConfigSnapshot,
} from "./arc/config";

// ── State shape ──────────────────────────────────────────────────────────────

export type ConfigState = {
    configKeys: Record<string, unknown>;
    configPresets: string[];
    currentPreset: string;
    configLoaded: boolean;
    lastConfigAck: ConfigAck | null;
};

export const configInitialState: ConfigState = {
    configKeys: {},
    configPresets: [],
    currentPreset: "",
    configLoaded: false,
    lastConfigAck: null,
};

// ── Message handler (call from onMessage in robot.ts) ───────────────────────

export function handleConfigMessage(
    identifier: string,
    payload: Uint8Array,
    set: (state: Partial<ConfigState>) => void
) {
    switch (identifier) {
        case "config_snapshot": {
            const snap = parseConfigSnapshot(payload);
            set({
                configKeys: snap.keys,
                configPresets: snap.presets,
                currentPreset: snap.currentPreset,
                configLoaded: true,
            });
            break;
        }
        case "config_key_changed": {
            const change = parseConfigKeyChanged(payload);
            set((prev: any) => ({
                configKeys: { ...prev.configKeys, [change.path]: change.value },
            }));
            break;
        }
        case "config_preset_list": {
            const list = parseConfigPresetList(payload);
            set({ configPresets: list.presets, currentPreset: list.currentPreset });
            break;
        }
        case "config_ack": {
            const ack = parseConfigAck(payload);
            set({ lastConfigAck: ack });
            // If the ack carries a new preset name it means load/save succeeded
            if (ack.success && ack.message) {
                set((prev: any) => ({ currentPreset: ack.message ?? prev.currentPreset }));
            }
            break;
        }
    }
}