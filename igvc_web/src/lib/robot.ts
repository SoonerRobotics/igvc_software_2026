import { create } from "zustand";
import { MessageAccumulator } from "./arc/accumulator";
import { MessageType } from "./arc/type";
import { ArcData } from "./messages/messages/arc";
import { ByteBuffer } from "flatbuffers";
import { ArcLog, buildArcData_Log, buildArcData_PropertyChanged, buildArcData_RobotPosition, buildArcData_RobotState } from "./arc/data";
import { VectornavReport } from "./messages/messages/vectornav-report";
import { ConfigState, configInitialState, handleConfigMessage } from "./robot-config";
import { encodeSetConfigKey, encodePresetName, encodeJson } from "./arc/encoders";
import { ArcCommandId } from "./messages/messages/arc/arc-command-id";
import { buildCommandReq } from "./arc/command";
import { MessageWrapper } from "./arc/wrapper";
import { MissionEnum, RobotModeEnum } from "./types";

type RobotState = ConfigState & {
    connected: boolean;
    path: string;
    connect: () => void;
    disconnect: () => void;
    setPath: (path: string) => void;

    // Config actions
    setConfigKey: (path: string, value: unknown) => void;
    loadPreset: (filename: string) => void;
    savePreset: (filename: string) => void;
    requestSnapshot: () => void;

    // Robot actions
    setMobility: (mobility: boolean) => void;
    setMission: (mission: number) => void;
    setMode: (mode: number) => void;
    calibrateHsvThreshold: () => void;

    // Data
    logs: ArcLog[];
    vectornav?: VectornavReport;
    current: number;
    voltage: number;
    state: {
        mobility: boolean;
        mission: MissionEnum;
        mode: RobotModeEnum;
    };
    waypointState?: {
        waypointSet: string;
        direction: string;
        currentIndex: number;
        finished: boolean;
        distanceMeters: number | null;
        bearingDegrees: number | null;
        target: { lat: number; lng: number } | null;
        waypoints: { lat: number; lng: number; index: number }[];
    };
};

let ws: WebSocket | null = null;
let connectTimeout: ReturnType<typeof setTimeout> | null = null;
let accumulator: MessageAccumulator | null = null;

function clearConnectTimeout() {
    if (connectTimeout) {
        clearTimeout(connectTimeout);
        connectTimeout = null;
    }
}

function getSavedPath() {
    if (typeof window === "undefined") return "ws://localhost:8080";
    const saved = localStorage.getItem("robotPath");
    if (saved) return saved;
    return `ws://${window.location.hostname}:8080`;
}

function savePath(path: string) {
    localStorage.setItem("robotPath", path);
}

function sendRaw(frame: Uint8Array) {
    if (ws?.readyState === WebSocket.OPEN) {
        ws.send(frame.slice().buffer);
    }
}

function onMessage(msg: MessageWrapper, set: (state: any) => void) {
    if (msg.type === MessageType.ArcData) {
        const data = ArcData.getRootAsArcData(new ByteBuffer(msg.toBytes()));
        const identifier = data.dataIdentifier()!;
        const payload = data.dataPayloadArray()!;

        if (identifier.startsWith("config_")) {
            handleConfigMessage(identifier, payload, set);
            return;
        }

        if (identifier === "log") {
            const log = buildArcData_Log(payload);
            set((state: any) => ({ logs: [...state.logs, log] }));

            // Keep only the most recent 100 logs to prevent memory issues
            set((state: any) => ({ logs: state.logs.slice(-100) }));

            // Sort by timestamp just in case they arrive out of order
            set((state: any) => ({ logs: state.logs.sort((a: ArcLog, b: ArcLog) => Number(a.timestamp) - Number(b.timestamp)) }));

            return;
        }

        if (identifier === "robot_state") {
            const d = buildArcData_RobotState(payload);
            set({
                state: {
                    mobility: d.MotionAllowed,
                    mission: d.Mission,
                    mode: d.Mode
                }
            });
            return;
        }

        if (identifier === "robot_position") {
            const d = buildArcData_RobotPosition(payload);
            console.log(d);
            // set({ state: {
            //     mobility: d.MotionAllowed,
            //     mission: d.Mission,
            //     mode: d.Mode
            // } });
            return;
        }

        if (identifier === "waypoint_state") {
            const json = JSON.parse(new TextDecoder().decode(payload));
            set({ waypointState: json });
            return;
        }

        if (identifier === "property_changed") {
            const d = buildArcData_PropertyChanged(payload);
            console.log("[robot] Property Changed", d);
            if (d.subsystem === "CurrentSensorSubsystem") {
                if (d.property === "current") set({ current: parseFloat(d.value) });
                if (d.property === "voltage") set({ voltage: parseFloat(d.value) * -1 });
            }
            return;
        }

        return;
    }

    if (msg.type === MessageType.VectorNav) {
        const data = VectornavReport.getRootAsVectornavReport(new ByteBuffer(msg.toBytes()));
        set({ vectornav: data });
        return;
    }

    console.log("[robot] message", msg.type, msg.length, "bytes", msg.toBytes());
}

export const useRobotStore = create<RobotState>((set, get) => ({
    connected: false,
    path: getSavedPath(),
    voltage: 0,
    current: 0,
    logs: [],
    ...configInitialState,
    state: {
        mobility: false,
        mission: MissionEnum.Autonav,
        mode: RobotModeEnum.Disabled
    },

    setPath: (path) => set({ path }),

    // ── Config actions ────────────────────────────────────────────────────
    setConfigKey: (path, value) =>
        sendRaw(buildCommandReq(ArcCommandId.SetConfigKey, encodeSetConfigKey(path, value))),

    loadPreset: (filename) =>
        sendRaw(buildCommandReq(ArcCommandId.LoadPreset, encodePresetName(filename))),

    savePreset: (filename) =>
        sendRaw(buildCommandReq(ArcCommandId.SavePreset, encodePresetName(filename))),

    requestSnapshot: () =>
        sendRaw(buildCommandReq(ArcCommandId.GetConfigSnapshot)),

    setMobility: (mobility: boolean) =>
        sendRaw(buildCommandReq(ArcCommandId.SetMobility, encodeJson({ mobility }))),

    setMission: (mission: number) =>
        sendRaw(buildCommandReq(ArcCommandId.SetMission, encodeJson({ mission }))),

    setMode: (mode: number) =>
        sendRaw(buildCommandReq(ArcCommandId.SetMode, encodeJson({ mode }))),

    calibrateHsvThreshold: () =>
        sendRaw(buildCommandReq(ArcCommandId.ToolsStartHsvCalibration)),

    // ── Connection ────────────────────────────────────────────────────────
    connect: () => {
        if (ws) {
            ws.onopen = null;
            ws.onclose = null;
            ws.onmessage = null;
            ws.onerror = null;
            ws.close();
            ws = null;
        }
        clearConnectTimeout();

        accumulator = new MessageAccumulator("little", (msg) => onMessage(msg, set));

        console.log("[robot] Connecting to", get().path);
        ws = new WebSocket(get().path);
        ws.binaryType = "arraybuffer";

        connectTimeout = setTimeout(() => {
            if (ws && ws.readyState !== WebSocket.OPEN) {
                ws.onopen = null;
                ws.onclose = null;
                ws.onerror = null;
                ws.close();
                ws = null;
                accumulator?.reset();
                set({ connected: false });
                setTimeout(() => get().connect(), 1000);
            }
        }, 2000);

        ws.onopen = () => {
            clearConnectTimeout();
            set({ connected: true });
            savePath(get().path);
        };

        ws.onclose = () => {
            clearConnectTimeout();
            set({ connected: false, configLoaded: false });
            ws = null;
            accumulator?.reset();
            setTimeout(() => get().connect(), 1000);
        };

        ws.onmessage = (event: MessageEvent) => {
            accumulator?.append(new Uint8Array(event.data as ArrayBuffer));
        };

        ws.onerror = (event: Event) => {
            console.error("WebSocket error:", event);
        };
    },

    disconnect: () => {
        clearConnectTimeout();
        if (ws) {
            ws.onopen = null;
            ws.onclose = null;
            ws.onerror = null;
            ws.close();
            ws = null;
        }
        accumulator?.reset();
        set({ connected: false, configLoaded: false });
    },
}));