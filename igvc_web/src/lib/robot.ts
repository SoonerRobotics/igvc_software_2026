import { create } from "zustand";
import { MessageWrapper } from "./arc/wrapper";
import { MessageAccumulator } from "./arc/accumulator";
import { MessageType } from "./arc/type";
import { ArcData } from "./messages/messages/arc";
import { ByteBuffer } from "flatbuffers";
import { ArcLog, buildArcData_Log, buildArcData_PropertyChanged } from "./arc/data";
import { VectornavReport } from "./messages/messages/vectornav-report";

type RobotState = {
    connected: boolean;
    path: string;
    connect: () => void;
    disconnect: () => void;
    setPath: (path: string) => void;

    // Data
    logs: ArcLog[];
    vectornav?: VectornavReport;
    current: number;
    voltage: number;
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
    // ensure client
    if (typeof window === "undefined") return "ws://localhost:8080";
    
    const saved = localStorage.getItem("robotPath");
    if (saved) return saved;

    // Default to same host as the web server, not localhost
    return `ws://${window.location.hostname}:8080`;
}

function savePath(path: string) {
    localStorage.setItem("robotPath", path);
}

function onMessage(msg: MessageWrapper, set: (state: any) => void) {
    if (msg.type === MessageType.ArcData)
    {
        const data = ArcData.getRootAsArcData(new ByteBuffer(msg.toBytes()));
        // console.log("[robot] ArcData", {
        //     identifier: data.dataIdentifier(),
        //     dataLength: data.dataPayloadLength(),
        //     timestamp: data.timestamp(),
        //     sequenceNumber: data.sequenceNumber()
        // });
        if (data.dataIdentifier() === "log")
        {
            const log = buildArcData_Log(data.dataPayloadArray()!);
            set((state: any) => ({ logs: [...state.logs, log] }));
        }

        if (data.dataIdentifier() == "property_changed")
        {
            const d = buildArcData_PropertyChanged(data.dataPayloadArray()!);
            console.log("[robot] Property Changed", d);

            if (d.subsystem == "CurrentSensorSubsystem")
            {
                if (d.property == "current")
                {
                    set({ current: parseFloat(d.value) });
                }

                if (d.property == "voltage")
                {
                    // parse and invert
                    set({ voltage: parseFloat(d.value) * -1 });
                }
            }
        }
        return;
    }

    if (msg.type == MessageType.VectorNav)
    {
        const data = VectornavReport.getRootAsVectornavReport(new ByteBuffer(msg.toBytes()));
        console.log("[robot] VectorNav", {
            latitude: data.latitude(),
            longitude: data.longitude(),
            yaw: data.yaw()
        });
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
    setPath: (path: string) => set({ path }),

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
            set({ connected: false });
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
        set({ connected: false });
    },

    // Data
    logs: [],
}));