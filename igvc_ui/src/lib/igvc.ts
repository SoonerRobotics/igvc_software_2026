import { readable } from "svelte/store";
import { handleImageMessage } from "./arc_handlers";
import { browser } from "$app/environment";
import { FlatBufferConverter, FlatBufferType, FlatBufferWrapper } from "./flat_wrapper";

type IGVCSocketData = {
    socket: WebSocket | null;
    status: "connected" | "disconnected" | "error";
    time: string;
}

export const igvcSocket = readable<IGVCSocketData>({ socket: null, status: "disconnected", time: "" }, (set) => {
    let socket: WebSocket;

    if (!browser) {
        return;
    }

    function connect() {
        console.log("Connecting to IGVC WebSocket...");

        socket = new WebSocket("ws://localhost:8080/");
        socket.binaryType = "arraybuffer";

        socket.onopen = () => {
            console.log("IGVC WebSocket connected");
            // set({ socket, status: "connected", time: "" });
        };

        socket.onclose = (event) => {
            console.log(`IGVC WebSocket disconnected: ${event.reason}`);
            set({ socket: null, status: "disconnected", time: "" });
            // Attempt to reconnect after a delay
            setTimeout(connect, 3000);
        };

        socket.onmessage = async (event) => {
            // Get the ArrayBuffer from the event
            let buffer: ArrayBuffer;
            if (event.data instanceof ArrayBuffer) {
                buffer = event.data;
            } else if (event.data instanceof Blob) {
                buffer = await event.data.arrayBuffer();
            } else {
                console.error("Unsupported message data type: ", typeof event.data);
                return;
            }

            try {
                const wrapper = FlatBufferWrapper.fromBuffer(new Uint8Array(buffer));
                switch (wrapper.messageType) {
                    case FlatBufferType.IMAGE_FRAME: {
                        handleImageMessage(FlatBufferConverter.asImageFrame(wrapper));
                    } break;

                    default:
                        console.warn("Unknown message type:", wrapper.messageType);
                        break;
                }
            } catch (error) {
                console.error("Failed to parse FlatBuffer message:", error);
                return;
            }
        };

        socket.onerror = (error) => {
            console.error("IGVC WebSocket error:", error);
            socket.close();
        };
    }

    console.log("Starting IGVC WebSocket connection...");

    connect();

    return () => {
        if (socket && socket.readyState === WebSocket.OPEN) {
            socket.close();
        }
    };
});