/**
 * CRC32 calculator (ITU-T V.42 / IEEE 802.3)
 * Same polynomial as .NET System.IO.Hashing.Crc32 (0xEDB88320).
 */

// Precompute the CRC-32 table once
const makeCrc32Table = (): Uint32Array => {
    const table = new Uint32Array(256);
    const poly = 0xedb88320;
    for (let i = 0; i < 256; i++) {
        let crc = i;
        for (let j = 0; j < 8; j++) {
            crc = (crc & 1) !== 0 ? (crc >>> 1) ^ poly : crc >>> 1;
        }
        table[i] = crc >>> 0;
    }
    return table;
};

const CRC32_TABLE = makeCrc32Table();

/**
 * Compute CRC32 for a complete Uint8Array in one shot.
 * @param data bytes to hash
 * @returns unsigned 32-bit CRC value
 */
export function crc32(data: Uint8Array): number {
    let crc = 0xffffffff;
    for (const byte of data) {
        const idx = (crc ^ byte) & 0xff;
        crc = (crc >>> 8) ^ CRC32_TABLE[idx];
    }
    return (crc ^ 0xffffffff) >>> 0;
}

/**
 * Incremental CRC32 class.
 * Use .append to feed chunks of data (like .NET’s Append).
 * Call .digest() for final CRC value.
 */
export class CRC32 {
    private crc: number = 0xffffffff;

    constructor() {
        this.reset();
    }

    /** Reset to initial state */
    reset(): this {
        this.crc = 0xffffffff;
        return this;
    }

    /** Append bytes (Uint8Array or string) */
    append(input: Uint8Array | string): this {
        let data: Uint8Array;
        if (typeof input === "string") {
            data = new TextEncoder().encode(input);
        } else {
            data = input;
        }

        for (const byte of data) {
            const idx = (this.crc ^ byte) & 0xff;
            this.crc = (this.crc >>> 8) ^ CRC32_TABLE[idx];
        }
        return this;
    }

    /** Get final CRC value (unsigned 32-bit) */
    digest(): number {
        return (this.crc ^ 0xffffffff) >>> 0;
    }

    /** Convenience: get hex string */
    hex(): string {
        return this.digest().toString(16).padStart(8, "0");
    }
}
