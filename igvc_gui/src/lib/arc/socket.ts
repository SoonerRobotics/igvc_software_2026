// src/lib/ws.ts
import { writable } from 'svelte/store';
import { MessageAccumulator } from './protocol/message_accumulator';
import { Endianness, makeMessageWrapperBB, MessageType, type MessageWrapper } from './protocol/types';
import { MessageWriter } from './protocol/message_writer';
import { constructCommandRequest } from './protocol/constructors';

export const connectionStatus = writable<'connecting' | 'open' | 'closed' | 'error'>('closed');

let socket: WebSocket | null = null;

// Decoder & Listener system

export type Decoder<T> = (msg: MessageWrapper) => T | null;

type Listener<T> = {
    type: MessageType;
    decode: Decoder<T>;
    filter?: (decoded: T) => boolean;
    callback: (msg: T) => void;
};

const listeners: Listener<any>[] = [];

export function subscribe<T>(
    type: MessageType,
    decode: Decoder<T>,
    callback: (msg: T) => void,
    filter?: (decoded: T) => boolean,
) {
    console.log(`Subscribed to message type ${MessageType[type]}`);
    const listener: Listener<T> = { type, decode, filter, callback: callback! };
    listeners.push(listener);

    return listener;
}

export function callCommand(type: number, payload?: Uint8Array) {
    const request = constructCommandRequest(type, payload);
    if (request.bb == null) {
        console.error("Failed to construct command request");
        return;
    }

    sendMessage(makeMessageWrapperBB(MessageType.CommandReq, request.bb));
}

export function unsubscribe(listener: Listener<any>) {
    const idx = listeners.indexOf(listener);
    if (idx !== -1) listeners.splice(idx, 1);
}

// Message Accumulator
const accumulator: MessageAccumulator = new MessageAccumulator(Endianness.Little, msg => {
    listeners.forEach(listener => {
        if (listener.type === msg.type) {
            const decoded = listener.decode(msg);
            if (decoded !== null) {
                if (!listener.filter || listener.filter(decoded)) {
                    listener.callback(decoded);
                }
            }
        }
    });
});

// Reconnect config
const MAX_RETRIES = 10;
let retries = 0;
let reconnectTimeout = 1000; // start at 1s

export function connect(url = 'ws://your-robot-endpoint') {
    connectionStatus.set('connecting');
    socket = new WebSocket(url);
    socket.binaryType = "arraybuffer";

    socket.onopen = () => {
        console.log('WS open');
        connectionStatus.set('open');

        // reset backoff
        retries = 0;
        reconnectTimeout = 1000;
    };

    socket.onmessage = (event) => {
        const data = event.data as ArrayBuffer;

        try {
            accumulator.append(new Uint8Array(data));
        } catch (err) {
            console.error('Error processing message', err);
        }
    };

    socket.onclose = (event) => {
        console.warn('WS closed', event);
        connectionStatus.set('closed');
        attemptReconnect(url);
    };

    socket.onerror = (err) => {
        console.error('WS error', err);
        connectionStatus.set('error');
        // treat this like a close
        socket?.close();
    };
}

function attemptReconnect(url: string) {
    if (retries >= MAX_RETRIES) {
        console.error('Max reconnect attempts reached');
        return;
    }

    retries++;
    reconnectTimeout *= 1.5; // exponential backoff

    console.log(`Reconnecting in ${reconnectTimeout}ms... (attempt ${retries})`);
    setTimeout(() => connect(url), reconnectTimeout);
}

export function disconnect() {
    socket?.close();
    socket = null;
    connectionStatus.set('closed');
}

export function sendMessage(data: MessageWrapper) {
    if (!socket || socket.readyState !== WebSocket.OPEN) {
        return;
    }

    const payload = MessageWriter.writeWrapper(data);
    socket.send(payload);
}
