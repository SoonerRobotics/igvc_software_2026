// lib/robot-config.ts
import {
    ConfigAck,
    parseConfigAck,
    parseConfigKeyChanged,
    parseConfigPresetList,
    parseConfigSnapshot,
} from "./arc/config";

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

export function handleConfigMessage(
    identifier: string,
    payload: Uint8Array,
    set: (state: any) => void
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
            set((prev: ConfigState) => ({
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
            if (ack.success && ack.message) {
                set((prev: ConfigState) => ({ currentPreset: ack.message ?? prev.currentPreset }));
            }
            break;
        }
    }
}