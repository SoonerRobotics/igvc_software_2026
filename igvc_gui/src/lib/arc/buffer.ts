export class CircularBuffer<T> {
    private buffer: (T | undefined)[];
    private capacity: number;

    constructor(capacity: number) {
        this.capacity = capacity;
        this.buffer = new Array(capacity).fill(undefined);
    }

    insert(item: T): void {
        this.buffer.unshift(item);
        if (this.buffer.length > this.capacity) {
            this.buffer.pop();
        }
    }

    get(index: number): T | undefined {
        return this.buffer[index];
    }

    getAll(): (T | undefined)[] {
        return [...this.buffer];
    }

    size(): number {
        return this.buffer.filter(item => item !== undefined).length;
    }

    clear(): void {
        this.buffer = new Array(this.capacity).fill(undefined);
    }
}