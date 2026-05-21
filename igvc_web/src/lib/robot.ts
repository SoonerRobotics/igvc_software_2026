import { create } from "zustand";
import { MessageWrapper } from "./arc/wrapper";
import { MessageAccumulator } from "./arc/accumulator";
import { MessageType } from "./arc/type";
import { ArcData } from "./messages/messages/arc";
import { ByteBuffer } from "flatbuffers";
import { ArcLog, buildArcData_Log } from "./arc/data";

type RobotState = {
    connected: boolean;
    path: string;
    connect: () => void;
    disconnect: () => void;
    setPath: (path: string) => void;

    // Data
    logs: ArcLog[];
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

function onMessage(msg: MessageWrapper, set: (state: any) => void) {
    if (msg.type === MessageType.ArcData)
    {
        const data = ArcData.getRootAsArcData(new ByteBuffer(msg.toBytes()));
        console.log("[robot] ArcData", {
            identifier: data.dataIdentifier(),
            dataLength: data.dataPayloadLength(),
            timestamp: data.timestamp(),
            sequenceNumber: data.sequenceNumber()
        });
        if (data.dataIdentifier() === "log")
        {
            const log = buildArcData_Log(data.dataPayloadArray()!);
            set((state: any) => ({ logs: [...state.logs, log] }));
        }
        return;
    }

    console.log("[robot] message", msg.type, msg.length, "bytes", msg.toBytes());
}

export const useRobotStore = create<RobotState>((set, get) => ({
    connected: false,
    path: "ws://192.168.1.83:8080",
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
    logs: []
}));