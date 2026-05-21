import { MessageType } from "./type";

export class MessageWrapper {
    readonly type: MessageType;
    readonly data: Uint8Array;
    readonly length: number;

    private constructor(type: MessageType, data: Uint8Array, length: number) {
        this.type = type;
        this.data = data;
        this.length = length;
    }

    toDataView(): DataView {
        return new DataView(this.data.buffer, this.data.byteOffset, this.length);
    }

    toBytes(): Uint8Array {
        return this.data.slice(0, this.length);
    }

    static from(type: MessageType, data: Uint8Array): MessageWrapper {
        return new MessageWrapper(type, data, data.byteLength);
    }

    static fromPooled(type: MessageType, data: Uint8Array, length: number): MessageWrapper {
        return new MessageWrapper(type, data, length);
    }
}