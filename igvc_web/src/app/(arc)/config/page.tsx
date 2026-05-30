"use client";

import { useState, useMemo, useCallback, useRef } from "react";
import { useRobotStore } from "@/lib/robot";
import Camera from "@/components/camera";

// ── Type detection ────────────────────────────────────────────────────────────

type ValueKind =
    | "boolean"
    | "number"
    | "string"
    | "velocity_mph"
    | "velocity_dps"
    | "color_range"
    | "point_array"
    | "byte_array"
    | "object"

function detectKind(value: unknown): ValueKind {
    if (typeof value === "boolean") return "boolean";
    if (typeof value === "number") return "number";
    if (typeof value === "string") {
        if (/^[A-Za-z0-9+/]+=*$/.test(value) && value.length % 4 === 0 && value.length > 0)
            return "byte_array";
        return "string";
    }
    if (Array.isArray(value) && value.length > 0 &&
        typeof value[0] === "object" && value[0] !== null &&
        "x" in value[0] && "y" in value[0])
        return "point_array";
    if (typeof value === "object" && value !== null) {
        if ("mph" in value) return "velocity_mph";
        if ("dps" in value) return "velocity_dps";
        if ("min" in value && "max" in value) return "color_range";
    }
    return "object";
}

function groupKeys(keys: Record<string, unknown>): Record<string, Record<string, unknown>> {
    const groups: Record<string, Record<string, unknown>> = {};
    for (const [path, value] of Object.entries(keys)) {
        const dot = path.lastIndexOf(".");
        const group = dot >= 0 ? path.slice(0, dot) : "(root)";
        const leaf = dot >= 0 ? path.slice(dot + 1) : path;
        if (!groups[group]) groups[group] = {};
        groups[group][leaf] = value;
    }
    return groups;
}

// ── Color helpers ─────────────────────────────────────────────────────────────

type HsvColor = { h: number; s: number; v: number };
type ColorRange = { min: HsvColor; max: HsvColor };

// OpenCV HSV: h 0-179, s 0-255, v 0-255
function hsvToCss({ h, s, v }: HsvColor): string {
    const hDeg = (h / 179) * 360;
    const sP = (s / 255) * 100;
    const lP = ((v / 255) * (1 - (s / 255) / 2)) * 100;
    return `hsl(${hDeg.toFixed(0)},${sP.toFixed(0)}%,${Math.max(lP, 8).toFixed(0)}%)`;
}

// ── HSV Range Slider ──────────────────────────────────────────────────────────

const HSV_CHANNELS = [
    { key: "h" as const, label: "Hue", max: 179, unit: "" },
    { key: "s" as const, label: "Saturation", max: 255, unit: "" },
    { key: "v" as const, label: "Value", max: 255, unit: "" },
];

function channelGradient(ch: "h" | "s" | "v", minColor: HsvColor, maxColor: HsvColor): string {
    if (ch === "h") {
        // Full hue spectrum
        const stops = Array.from({ length: 13 }, (_, i) => {
            const hVal = Math.round((i / 12) * 179);
            return hsvToCss({ h: hVal, s: 200, v: 220 });
        });
        return `linear-gradient(to right, ${stops.join(", ")})`;
    }
    if (ch === "s") {
        const lo = hsvToCss({ ...minColor, s: 0 });
        const hi = hsvToCss({ ...minColor, s: 255 });
        return `linear-gradient(to right, ${lo}, ${hi})`;
    }
    // v
    const lo = hsvToCss({ ...minColor, v: 0 });
    const hi = hsvToCss({ ...minColor, v: 255 });
    return `linear-gradient(to right, ${lo}, ${hi})`;
}

function HsvRangeSliders({ draft, onChange }: {
    draft: ColorRange;
    onChange: (next: ColorRange) => void;
}) {
    return (
        <div className="space-y-3">
            {HSV_CHANNELS.map(({ key, label, max }) => {
                const lo = draft.min[key];
                const hi = draft.max[key];
                const pLo = (lo / max) * 100;
                const pHi = (hi / max) * 100;
                const grad = channelGradient(key, draft.min, draft.max);

                function setLo(v: number) {
                    onChange({ ...draft, min: { ...draft.min, [key]: Math.min(v, hi) } });
                }
                function setHi(v: number) {
                    onChange({ ...draft, max: { ...draft.max, [key]: Math.max(v, lo) } });
                }

                return (
                    <div key={key} className="space-y-1">
                        <div className="flex items-center justify-between">
                            <span className="text-zinc-400 text-xs font-mono">{label}</span>
                            <span className="text-zinc-500 text-xs font-mono tabular-nums">{lo} – {hi}</span>
                        </div>

                        {/* Track */}
                        <div className="relative h-5 flex items-center">
                            {/* Gradient background */}
                            <div className="absolute inset-x-0 h-2 rounded-full" style={{ background: grad }} />

                            {/* Active range highlight */}
                            <div
                                className="absolute h-2 rounded-full bg-white/20 border border-white/30"
                                style={{ left: `${pLo}%`, right: `${100 - pHi}%` }}
                            />

                            {/* Lo thumb */}
                            <input
                                type="range" min={0} max={max} value={lo}
                                onChange={e => setLo(Number(e.target.value))}
                                className="absolute inset-0 w-full opacity-0 cursor-pointer h-5"
                                style={{ zIndex: lo > max * 0.9 ? 4 : 3 }}
                            />

                            {/* Hi thumb */}
                            <input
                                type="range" min={0} max={max} value={hi}
                                onChange={e => setHi(Number(e.target.value))}
                                className="absolute inset-0 w-full opacity-0 cursor-pointer h-5"
                                style={{ zIndex: 4 }}
                            />

                            {/* Visible thumbs */}
                            <div
                                className="absolute w-3.5 h-3.5 rounded-full border-2 border-white shadow-md pointer-events-none"
                                style={{ left: `calc(${pLo}% - 7px)`, background: hsvToCss({ ...draft.min, [key]: lo }), zIndex: 5 }}
                            />
                            <div
                                className="absolute w-3.5 h-3.5 rounded-full border-2 border-white shadow-md pointer-events-none"
                                style={{ left: `calc(${pHi}% - 7px)`, background: hsvToCss({ ...draft.max, [key]: hi }), zIndex: 5 }}
                            />
                        </div>

                        {/* Number inputs */}
                        <div className="flex gap-2">
                            <input type="number" min={0} max={max} value={lo}
                                onChange={e => setLo(Number(e.target.value))}
                                className="w-16 font-mono text-xs px-1.5 py-0.5 rounded border border-zinc-700 bg-zinc-900 text-white focus:outline-none focus:border-sky-600 tabular-nums"
                            />
                            <div className="flex-1" />
                            <input type="number" min={0} max={max} value={hi}
                                onChange={e => setHi(Number(e.target.value))}
                                className="w-16 font-mono text-xs px-1.5 py-0.5 rounded border border-zinc-700 bg-zinc-900 text-white focus:outline-none focus:border-sky-600 tabular-nums text-right"
                            />
                        </div>
                    </div>
                );
            })}
        </div>
    );
}

// ── Color range editor ────────────────────────────────────────────────────────

// Identifies which config keys should show a camera feed below them
const CAMERA_KEYS: Record<string, string> = {
    "vision.ground_threshold": "combined_filtered",
    "vision.yellow_threshold": "combined_filtered",
    "vision.left_source_points": "combined_debug"
};

function ColorRangeEditor({ path, value, onLiveChange, onRevert }: {
    path: string;
    value: ColorRange;
    onLiveChange: (path: string, v: ColorRange) => void;
    onRevert: () => void;
}) {
    const [draft, setDraft] = useState<ColorRange>(value);
    const cameraId = CAMERA_KEYS[path];

    function handleChange(next: ColorRange) {
        setDraft(next);
        onLiveChange(path, next);
    }

    return (
        <div className="border border-zinc-700 rounded-lg p-3 bg-zinc-950 space-y-3 flex flex-col gap-2">
            {/* Swatches */}
            <div className="flex items-center gap-3">
                <div className="flex items-center gap-1.5">
                    <span className="w-4 h-4 rounded border border-zinc-600" style={{ background: hsvToCss(draft.min) }} />
                    <span className="text-zinc-600 text-xs font-mono">min</span>
                </div>
                <div className="h-px flex-1 bg-zinc-800" />
                <div className="flex items-center gap-1.5">
                    <span className="text-zinc-600 text-xs font-mono">max</span>
                    <span className="w-4 h-4 rounded border border-zinc-600" style={{ background: hsvToCss(draft.max) }} />
                </div>
            </div>

            <p className="text-zinc-700 text-xs font-mono">H 0–179 · S,V 0–255</p>

            <HsvRangeSliders draft={draft} onChange={handleChange} />

            <button
                onClick={onRevert}
                className="text-xs px-2 py-0.5 rounded bg-zinc-800 text-zinc-400 hover:bg-zinc-700 border border-zinc-600 font-mono"
            >
                ↩ revert
            </button>

            {cameraId && (
                <div className="pt-1 w-full">
                    <Camera id={cameraId} />
                </div>
            )}
        </div>
    );
}

// ── Simple editors ────────────────────────────────────────────────────────────

function BooleanEditor({ value, onChange }: { value: boolean; onChange: (v: boolean) => void }) {
    return (
        <button
            onClick={() => onChange(!value)}
            className={`inline-flex items-center gap-1.5 px-2 py-0.5 rounded text-xs font-mono font-medium transition-colors ${value
                    ? "bg-emerald-950 text-emerald-400 border border-emerald-800 hover:bg-emerald-900"
                    : "bg-zinc-900 text-zinc-500 border border-zinc-700 hover:bg-zinc-800"
                }`}
        >
            <span className={`w-1.5 h-1.5 rounded-full ${value ? "bg-emerald-400" : "bg-zinc-600"}`} />
            {value ? "true" : "false"}
        </button>
    );
}

function NumberEditor({ value, onChange }: { value: number; onChange: (v: number) => void }) {
    const [editing, setEditing] = useState(false);
    const [draft, setDraft] = useState(String(value));

    if (!editing) return (
        <button onClick={() => { setDraft(String(value)); setEditing(true); }}
            className="font-mono text-xs px-2 py-0.5 rounded border bg-zinc-900 border-zinc-700 text-zinc-300 hover:border-zinc-500 hover:text-white transition-colors">
            {value}
        </button>
    );
    return (
        <span className="inline-flex items-center gap-1">
            <input autoFocus type="number" value={draft}
                onChange={e => setDraft(e.target.value)}
                onKeyDown={e => {
                    if (e.key === "Enter") { onChange(Number(draft)); setEditing(false); }
                    if (e.key === "Escape") setEditing(false);
                }}
                className="font-mono text-xs px-2 py-0.5 rounded border border-sky-600 bg-zinc-900 text-white w-28 focus:outline-none"
            />
            <button onClick={() => { onChange(Number(draft)); setEditing(false); }} className="text-xs px-1.5 py-0.5 rounded bg-sky-800 text-sky-200 hover:bg-sky-700 border border-sky-600">✓</button>
            <button onClick={() => setEditing(false)} className="text-xs px-1.5 py-0.5 rounded bg-zinc-800 text-zinc-400 hover:bg-zinc-700 border border-zinc-600">✕</button>
        </span>
    );
}

function StringEditor({ value, onChange }: { value: string; onChange: (v: string) => void }) {
    const [editing, setEditing] = useState(false);
    const [draft, setDraft] = useState(value);

    if (!editing) return (
        <button onClick={() => { setDraft(value); setEditing(true); }}
            className="font-mono text-xs px-2 py-0.5 rounded border bg-zinc-900 border-zinc-700 text-zinc-300 hover:border-zinc-500 hover:text-white transition-colors max-w-48 truncate">
            {value || <span className="text-zinc-600 italic">empty</span>}
        </button>
    );
    return (
        <span className="inline-flex items-center gap-1">
            <input autoFocus value={draft}
                onChange={e => setDraft(e.target.value)}
                onKeyDown={e => {
                    if (e.key === "Enter") { onChange(draft); setEditing(false); }
                    if (e.key === "Escape") setEditing(false);
                }}
                className="font-mono text-xs px-2 py-0.5 rounded border border-sky-600 bg-zinc-900 text-white w-40 focus:outline-none"
            />
            <button onClick={() => { onChange(draft); setEditing(false); }} className="text-xs px-1.5 py-0.5 rounded bg-sky-800 text-sky-200 hover:bg-sky-700 border border-sky-600">✓</button>
            <button onClick={() => setEditing(false)} className="text-xs px-1.5 py-0.5 rounded bg-zinc-800 text-zinc-400 hover:bg-zinc-700 border border-zinc-600">✕</button>
        </span>
    );
}

function UnitNumberEditor({ value, unit, onChange, step = 0.01, decimals = 3 }: {
    value: number; unit: string; onChange: (v: number) => void; step?: number; decimals?: number;
}) {
    const [editing, setEditing] = useState(false);
    const [draft, setDraft] = useState(value.toFixed(decimals));

    if (!editing) return (
        <button onClick={() => { setDraft(value.toFixed(decimals)); setEditing(true); }}
            className="font-mono text-xs px-2 py-0.5 rounded border bg-zinc-900 border-zinc-700 text-zinc-300 hover:border-zinc-500 hover:text-white transition-colors inline-flex items-center gap-1.5">
            {value.toFixed(decimals)} <span className="text-zinc-600">{unit}</span>
        </button>
    );
    return (
        <span className="inline-flex items-center gap-1">
            <input autoFocus type="number" value={draft} step={step}
                onChange={e => setDraft(e.target.value)}
                onKeyDown={e => {
                    if (e.key === "Enter") { onChange(Number(draft)); setEditing(false); }
                    if (e.key === "Escape") setEditing(false);
                }}
                className="font-mono text-xs px-2 py-0.5 rounded border border-sky-600 bg-zinc-900 text-white w-28 focus:outline-none"
            />
            <span className="text-zinc-500 text-xs font-mono">{unit}</span>
            <button onClick={() => { onChange(Number(draft)); setEditing(false); }} className="text-xs px-1.5 py-0.5 rounded bg-sky-800 text-sky-200 hover:bg-sky-700 border border-sky-600">✓</button>
            <button onClick={() => setEditing(false)} className="text-xs px-1.5 py-0.5 rounded bg-zinc-800 text-zinc-400 hover:bg-zinc-700 border border-zinc-600">✕</button>
        </span>
    );
}

type Point2f = { x: number; y: number };

function PointArrayEditor({ value, onChange }: { value: Point2f[]; onChange: (v: Point2f[]) => void }) {
    const [open, setOpen] = useState(false);
    const [draft, setDraft] = useState(value);

    function setCoord(i: number, axis: "x" | "y", val: number) {
        setDraft(d => d.map((p, j) => j === i ? { ...p, [axis]: val } : p));
    }

    if (!open) return (
        <button onClick={() => { setDraft(value); setOpen(true); }}
            className="font-mono text-xs px-2 py-0.5 rounded border border-zinc-700 bg-zinc-900 text-zinc-400 hover:border-zinc-500 hover:text-white transition-colors">
            [{value.map(p => `(${p.x},${p.y})`).join(", ")}]
        </button>
    );

    return (
        <div className="border border-zinc-700 rounded-lg p-3 bg-zinc-950 space-y-1.5">
            {draft.map((pt, i) => (
                <div key={i} className="flex items-center gap-2">
                    <span className="text-zinc-600 text-xs font-mono w-4">{i}</span>
                    {(["x", "y"] as const).map(axis => (
                        <label key={axis} className="flex items-center gap-1">
                            <span className="text-zinc-500 text-xs font-mono">{axis}</span>
                            <input type="number" value={pt[axis]}
                                onChange={e => setCoord(i, axis, Number(e.target.value))}
                                className="w-16 font-mono text-xs px-1.5 py-0.5 rounded border border-zinc-700 bg-zinc-900 text-white focus:outline-none focus:border-sky-600"
                            />
                        </label>
                    ))}
                </div>
            ))}
            <div className="flex gap-2 pt-1">
                <button onClick={() => { onChange(draft); setOpen(false); }} className="text-xs px-2 py-0.5 rounded bg-sky-800 text-sky-200 hover:bg-sky-700 border border-sky-600 font-mono">save</button>
                <button onClick={() => { setDraft(value); setOpen(false); }} className="text-xs px-2 py-0.5 rounded bg-zinc-800 text-zinc-400 hover:bg-zinc-700 border border-zinc-600 font-mono">cancel</button>
            </div>
        </div>
    );
}

function ByteArrayDisplay({ value }: { value: string }) {
    const bytes = useMemo(() => {
        try { return Array.from(atob(value)).map(c => c.charCodeAt(0)); }
        catch { return []; }
    }, [value]);
    return (
        <span className="font-mono text-xs px-2 py-0.5 rounded border border-zinc-800 bg-zinc-900 text-zinc-500">
            {bytes.map(b => b.toString(16).padStart(2, "0")).join(" ")}
            <span className="text-zinc-700 ml-1">({bytes.length}B)</span>
        </span>
    );
}

// ── Unified value cell ────────────────────────────────────────────────────────

function ConfigValue({ path, value, onSave, onLiveChange }: {
    path: string;
    value: unknown;
    onSave: (path: string, value: unknown) => void;
    onLiveChange: (path: string, value: unknown) => void;
}) {
    const kind = detectKind(value);

    // For color_range: expand inline (not hidden behind a toggle button)
    if (kind === "color_range") {
        return (
            <ColorRangeEditor
                path={path}
                value={value as ColorRange}
                onLiveChange={onLiveChange}
                onRevert={() => onSave(path, value)}
            />
        );
    }

    switch (kind) {
        case "boolean":
            return <BooleanEditor value={value as boolean} onChange={v => onSave(path, v)} />;
        case "number":
            return <NumberEditor value={value as number} onChange={v => onSave(path, v)} />;
        case "string":
            return <StringEditor value={value as string} onChange={v => onSave(path, v)} />;
        case "byte_array":
            return <ByteArrayDisplay value={value as string} />;
        case "velocity_mph":
            return <UnitNumberEditor value={(value as { mph: number }).mph} unit="mph"
                onChange={v => onSave(path, { mph: v })} step={0.1} decimals={3} />;
        case "velocity_dps":
            return <UnitNumberEditor value={(value as { dps: number }).dps} unit="°/s"
                onChange={v => onSave(path, { dps: v })} step={1} decimals={1} />;
        case "point_array":
            return <PointArrayEditor value={value as Point2f[]} onChange={v => onSave(path, v)} />;
        default:
            return (
                <span className="font-mono text-xs px-2 py-0.5 rounded border border-zinc-800 bg-zinc-900 text-zinc-600 max-w-xs truncate block">
                    {JSON.stringify(value)}
                </span>
            );
    }
}

// ── Config group ──────────────────────────────────────────────────────────────

function ConfigGroup({ group, keys, onSave, onLiveChange }: {
    group: string;
    keys: Record<string, unknown>;
    onSave: (path: string, value: unknown) => void;
    onLiveChange: (path: string, value: unknown) => void;
}) {
    const [open, setOpen] = useState(true);
    const entries = Object.entries(keys);

    return (
        <div className="border border-zinc-800 rounded-lg overflow-hidden">
            <button onClick={() => setOpen(o => !o)}
                className="w-full flex items-center justify-between px-4 py-2.5 bg-zinc-900 hover:bg-zinc-800 transition-colors text-left">
                <span className="font-mono text-xs text-zinc-400 tracking-widest uppercase">{group}</span>
                <span className="text-zinc-600 text-xs">{open ? "▴" : "▾"} {entries.length} key{entries.length !== 1 ? "s" : ""}</span>
            </button>
            {open && (
                <div className="divide-y divide-zinc-800/60">
                    {entries.map(([leaf, value]) => {
                        const fullPath = group === "(root)" ? leaf : `${group}.${leaf}`;
                        const isExpanded = detectKind(value) === "color_range";
                        return (
                            <div key={leaf} className={`px-4 py-2.5 gap-4 hover:bg-zinc-900/40 ${isExpanded ? "" : "flex items-start justify-between"}`}>
                                <span className={`font-mono text-sm text-zinc-300 min-w-0 truncate ${isExpanded ? "block mb-2" : "pt-0.5"}`} title={fullPath}>
                                    {leaf}
                                </span>
                                <div className={isExpanded ? "" : "flex-shrink-0"}>
                                    <ConfigValue path={fullPath} value={value} onSave={onSave} onLiveChange={onLiveChange} />
                                </div>
                            </div>
                        );
                    })}
                </div>
            )}
        </div>
    );
}

// ── Page ──────────────────────────────────────────────────────────────────────

export default function ConfigPage() {
    const connected = useRobotStore(s => s.connected);
    const configLoaded = useRobotStore(s => s.configLoaded);
    const configKeys = useRobotStore(s => s.configKeys);
    const presets = useRobotStore(s => s.configPresets);
    const currentPreset = useRobotStore(s => s.currentPreset);
    const lastAck = useRobotStore(s => s.lastConfigAck);
    const setConfigKey = useRobotStore(s => s.setConfigKey);
    const loadPreset = useRobotStore(s => s.loadPreset);
    const savePreset = useRobotStore(s => s.savePreset);
    const requestSnapshot = useRobotStore(s => s.requestSnapshot);

    const [search, setSearch] = useState("");
    const [saveInput, setSaveInput] = useState("");
    const [savingAs, setSavingAs] = useState(false);

    // Throttle live updates so we don't flood the WebSocket on every slider tick
    const liveTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
    const handleLiveChange = useCallback((path: string, value: unknown) => {
        if (liveTimer.current) clearTimeout(liveTimer.current);
        liveTimer.current = setTimeout(() => setConfigKey(path, value), 40);
    }, [setConfigKey]);

    const groups = useMemo(() => groupKeys(configKeys), [configKeys]);

    const filteredGroups = useMemo(() => {
        if (!search.trim()) return groups;
        const q = search.toLowerCase();
        const result: typeof groups = {};
        for (const [group, keys] of Object.entries(groups)) {
            const filtered = Object.fromEntries(
                Object.entries(keys).filter(([leaf, value]) =>
                    leaf.toLowerCase().includes(q) ||
                    group.toLowerCase().includes(q) ||
                    JSON.stringify(value).toLowerCase().includes(q)
                )
            );
            if (Object.keys(filtered).length > 0) result[group] = filtered;
        }
        return result;
    }, [groups, search]);

    const totalKeys = Object.keys(configKeys).length;
    const filteredCount = Object.values(filteredGroups).reduce((n, g) => n + Object.keys(g).length, 0);

    if (!connected) return (
        <div className="flex items-center justify-center h-64 text-zinc-600 text-sm font-mono">not connected</div>
    );
    if (!configLoaded) return (
        <div className="flex items-center justify-center h-64 text-zinc-600 text-sm font-mono gap-2">
            <span className="inline-block w-2 h-2 rounded-full bg-zinc-600 animate-pulse" />
            waiting for config…
        </div>
    );

    return (
        <div className="max-w-3xl mx-auto px-4 py-8 space-y-6">
            {/* Header */}
            <div className="flex items-start justify-between gap-4">
                <div>
                    <h1 className="text-white text-lg font-mono font-medium tracking-tight">Configuration</h1>
                    <p className="text-zinc-500 text-xs font-mono mt-0.5">
                        {totalKeys} key{totalKeys !== 1 ? "s" : ""}
                        {search && ` · ${filteredCount} matching`}
                    </p>
                </div>
                <button onClick={requestSnapshot} className="text-xs font-mono px-3 py-1.5 rounded border border-zinc-700 text-zinc-400 hover:text-white hover:border-zinc-500 transition-colors">
                    ↺ refresh
                </button>
            </div>

            {/* Ack toast */}
            {lastAck && (
                <div className={`text-xs font-mono px-3 py-2 rounded border ${lastAck.success
                        ? "bg-emerald-950 border-emerald-800 text-emerald-400"
                        : "bg-red-950 border-red-800 text-red-400"
                    }`}>
                    {lastAck.success ? "✓" : "✕"} {lastAck.message ?? (lastAck.success ? "ok" : "error")}
                </div>
            )}

            {/* Preset bar */}
            <div className="border border-zinc-800 rounded-lg p-4 space-y-3">
                <div className="flex items-center justify-between">
                    <span className="text-xs font-mono text-zinc-400 uppercase tracking-widest">Preset</span>
                    <span className="text-xs font-mono text-sky-400 bg-sky-950 border border-sky-800 px-2 py-0.5 rounded">
                        {currentPreset || "—"}
                    </span>
                </div>
                {presets.length > 0 && (
                    <div className="flex flex-wrap gap-2">
                        {presets.map(preset => (
                            <button key={preset} onClick={() => loadPreset(preset)}
                                className={`text-xs font-mono px-2.5 py-1 rounded border transition-colors ${preset === currentPreset
                                        ? "border-sky-700 bg-sky-950 text-sky-300"
                                        : "border-zinc-700 bg-zinc-900 text-zinc-400 hover:border-zinc-500 hover:text-white"
                                    }`}>
                                {preset}
                            </button>
                        ))}
                    </div>
                )}
                {savingAs ? (
                    <div className="flex items-center gap-2">
                        <input autoFocus value={saveInput}
                            onChange={e => setSaveInput(e.target.value)}
                            onKeyDown={e => {
                                if (e.key === "Enter" && saveInput.trim()) { savePreset(saveInput.trim()); setSavingAs(false); setSaveInput(""); }
                                if (e.key === "Escape") setSavingAs(false);
                            }}
                            placeholder="filename.json"
                            className="font-mono text-xs px-2 py-1 rounded border border-zinc-600 bg-zinc-900 text-white flex-1 focus:outline-none focus:border-sky-500"
                        />
                        <button onClick={() => { if (saveInput.trim()) { savePreset(saveInput.trim()); setSavingAs(false); setSaveInput(""); } }}
                            className="text-xs px-2 py-1 rounded bg-sky-800 text-sky-200 hover:bg-sky-700 border border-sky-600 font-mono">save</button>
                        <button onClick={() => setSavingAs(false)}
                            className="text-xs px-2 py-1 rounded bg-zinc-800 text-zinc-400 hover:bg-zinc-700 border border-zinc-600 font-mono">cancel</button>
                    </div>
                ) : (
                    <button onClick={() => { setSaveInput(currentPreset || ""); setSavingAs(true); }}
                        className="text-xs font-mono text-zinc-500 hover:text-zinc-300 transition-colors">
                        + save as new preset…
                    </button>
                )}
            </div>

            {/* Search */}
            <input value={search} onChange={e => setSearch(e.target.value)} placeholder="filter keys…"
                className="w-full font-mono text-sm px-3 py-2 rounded-lg border border-zinc-800 bg-zinc-900 text-white placeholder-zinc-600 focus:outline-none focus:border-zinc-600"
            />

            {/* Groups */}
            <div className="space-y-2">
                {Object.entries(filteredGroups).map(([group, keys]) => (
                    <ConfigGroup key={group} group={group} keys={keys} onSave={setConfigKey} onLiveChange={handleLiveChange} />
                ))}
                {Object.keys(filteredGroups).length === 0 && (
                    <p className="text-center text-zinc-600 text-sm font-mono py-8">no keys match</p>
                )}
            </div>
        </div>
    );
}