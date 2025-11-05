import { readable } from "svelte/store";

type IGVCSocketData = {
    socket: WebSocket | null;
    status: "connected" | "disconnected" | "error";
    time: string;
}

export const igvcSocket = readable<IGVCSocketData>({ socket: null, status: "disconnected", time: "" }, (set) => {
    let socket: WebSocket;

    function connect() {
        console.log("Connecting to IGVC WebSocket...");

        socket = new WebSocket("ws://localhost:9002/igvc");

        socket.onopen = () => {
            console.log("IGVC WebSocket connected");
            set({ socket, status: "connected", time: "" });
        };

        socket.onclose = (event) => {
            console.log(`IGVC WebSocket disconnected: ${event.reason}`);
            set({ socket: null, status: "disconnected", time: "" });
            // Attempt to reconnect after a delay
            setTimeout(connect, 5000);
        };

        socket.onmessage = (event) => {
            const data = JSON.parse(event.data);
            set({ socket, status: "connected", time: data.time });
        };

        socket.onerror = (error) => {
            console.error("IGVC WebSocket error:", error);
            socket.close();
        };
    }

    connect();

    return () => {
        if (socket && socket.readyState === WebSocket.OPEN) {
            socket.close();
        }
    };
});