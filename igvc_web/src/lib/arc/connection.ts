import { writable, derived, get } from 'svelte/store';
import { MessageAccumulator } from '$lib/arc/protocol/accumulator';
import { MessageWriter } from '$lib/arc/protocol/writer';
import { MessageType, Endianness, type MessageWrapper } from '$lib/arc/protocol/types';
import decoder from '$lib/arc/protocol/decoder';

export type ConnectionStatus = 'disconnected' | 'connecting' | 'connected' | 'error';
export const connectionStatus = writable<ConnectionStatus>('disconnected');
export const connectionError = writable<string | null>(null);
export const latency = writable<number>(0);

// Data Stores

export const vectornav = writable<{
    lat: number;
    lng: number;
    numSats: number;
    fix: number;

    roll: number;
    pitch: number;
    yaw: number;
} | null>(null);
export const metricHistory = writable<unknown | null>(null);
export const latestMetricSample = writable<unknown | null>(null);
export const latestImageFrame = writable<Uint8Array | null>(null);
export const isConnected = derived(connectionStatus, $s => $s === 'connected');

// Websocket Stuff

let ws: WebSocket | null = null;
let accumulator: MessageAccumulator | null = null;
let reconnectTimer: ReturnType<typeof setTimeout> | null = null;
let pingTimer: ReturnType<typeof setInterval> | null = null;
let pingStart = 0;
const RECONNECT_DELAY_MS = 2000;

function clearTimers() {
    if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
    if (pingTimer) { clearInterval(pingTimer); pingTimer = null; }
}

function handleMessage(msg: MessageWrapper) {
    switch (msg.type) {
        case MessageType.CapabilityAck: {
            decoder.capability_ack(msg);
            break;
        }
        case MessageType.Gps: {
            const report = decoder.vectornav(msg);
            vectornav.set({
                lat: report.latitude(),
                lng: report.longitude(),
                numSats: report.numSatellites(),
                fix: report.fixQuality(),
                roll: report.roll(),
                pitch: report.pitch(),
                yaw: report.yaw(),
            });
            break;
        }
        default:
            console.warn('[ws] Unknown message type:', msg.type);
    }
}

export function connect() {
    if (ws && ws.readyState !== WebSocket.CLOSED) {
        ws.close();
    }

    clearTimers();
    connectionStatus.set('connecting');
    connectionError.set(null);

    ws = new WebSocket(get(wsUrl));
    ws.binaryType = 'arraybuffer';

    accumulator = new MessageAccumulator(
        Endianness.Little,
        handleMessage
    );

    ws.onopen = () => {
        connectionStatus.set('connected');
        connectionError.set(null);

        pingTimer = setInterval(() => {
            // pingStart = performance.now();
        }, 2000);
    };

    ws.onmessage = (event: MessageEvent<ArrayBuffer>) => {
        try {
            accumulator?.append(new Uint8Array(event.data));
        } catch (err) {
            console.error('[ws] Accumulator error:', err);
            connectionError.set(String(err));
        }
    };

    ws.onerror = () => {
        connectionStatus.set('error');
        connectionError.set('WebSocket error');
    };

    ws.onclose = (event) => {
        clearTimers();
        if (get(connectionStatus) !== 'error') {
            connectionStatus.set('disconnected');
        }
        console.log(`[ws] Closed (code ${event.code}). Reconnecting in ${RECONNECT_DELAY_MS}ms...`);
        reconnectTimer = setTimeout(() => connect(), RECONNECT_DELAY_MS);
    };
}

export function disconnect() {
    clearTimers();
    ws?.close();
    ws = null;
    accumulator = null;
    connectionStatus.set('disconnected');
}

export function send(msg: MessageWrapper) {
    if (ws == null || ws.readyState !== WebSocket.OPEN) {
        console.warn('[ws] Cannot send — not connected');
        return;
    }

    const payload = MessageWriter.writeWrapper(msg);
    ws.send(payload.slice(0, payload.length));
}

export const wsUrl = writable<string>('ws://127.0.0.1:8080/');