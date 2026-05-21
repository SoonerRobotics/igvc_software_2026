import { create } from "zustand";

type RobotState = {
    // General State
    connected: boolean;

    // Robot State

    // Actions
    connect: (url: string) => void;
    disconnect: () => void;

    // Commands
}

let ws: WebSocket | null = null;
export const useRobotStore = create<RobotState>((set, get) => ({
    connected: false,
    connect: (url: string) => {
        if (ws) {
            ws.close();
        }
        ws = new WebSocket(url);
        ws.binaryType = "arraybuffer";
        ws.onopen = () => {
            set({ connected: true });
        };
        ws.onclose = () => {
            set({ connected: false });
            setTimeout(() => {
                if (ws) {
                    ws.close();
                    ws = null;
                }

                get().connect(url);
            }, 1000);
        };

        // Messages
        ws.onmessage = (event) => {
            // TODO: Handle incoming messages from the robot
        };
    },
    disconnect: () => {
        if (ws) {
            ws.close();
            ws = null;
        }
    },
}));