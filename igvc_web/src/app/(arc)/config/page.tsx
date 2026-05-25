"use client";

import { useState, useMemo } from "react";
import { useRobotStore } from "@/lib/robot";

// ── Helpers ──────────────────────────────────────────────────────────────────

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

function valueType(value: unknown): "boolean" | "number" | "string" {
    if (typeof value === "boolean") return "boolean";
    if (typeof value === "number") return "number";
    return "string";
}

// ── Sub-components ───────────────────────────────────────────────────────────

function ConfigValue({
    path,
    leafKey,
    value,
    onSave,
}: {
    path: string;
    leafKey: string;
    value: unknown;
    onSave: (path: string, value: unknown) => void;
}) {
    const type = valueType(value);
    const [editing, setEditing] = useState(false);
    const [draft, setDraft] = useState(String(value));
    const [pending, setPending] = useState(false);

    function commit() {
        let parsed: unknown = draft;
        if (type === "boolean") parsed = draft === "true";
        else if (type === "number") parsed = Number(draft);
        setPending(true);
        onSave(path, parsed);
        setTimeout(() => setPending(false), 800);
        setEditing(false);
    }

    function cancel() {
        setDraft(String(value));
        setEditing(false);
    }

    if (type === "boolean") {
        const bool = value as boolean;
        return (
            <button
                onClick={() => onSave(path, !bool)}
                className={`inline-flex items-center gap-1.5 px-2 py-0.5 rounded text-xs font-mono font-medium transition-colors ${bool
                        ? "bg-emerald-950 text-emerald-400 border border-emerald-800 hover:bg-emerald-900"
                        : "bg-zinc-900 text-zinc-500 border border-zinc-700 hover:bg-zinc-800"
                    }`}
            >
                <span className={`w-1.5 h-1.5 rounded-full ${bool ? "bg-emerald-400" : "bg-zinc-600"}`} />
                {bool ? "true" : "false"}
            </button>
        );
    }

    if (!editing) {
        return (
            <button
                onClick={() => { setDraft(String(value)); setEditing(true); }}
                className={`font-mono text-xs px-2 py-0.5 rounded border transition-colors text-left ${pending
                        ? "bg-amber-950 border-amber-700 text-amber-300"
                        : "bg-zinc-900 border-zinc-700 text-zinc-300 hover:border-zinc-500 hover:text-white"
                    }`}
            >
                {String(value)}
            </button>
        );
    }

    return (
        <span className="inline-flex items-center gap-1">
            <input
                autoFocus
                value={draft}
                onChange={e => setDraft(e.target.value)}
                onKeyDown={e => { if (e.key === "Enter") commit(); if (e.key === "Escape") cancel(); }}
                className="font-mono text-xs px-2 py-0.5 rounded border border-sky-600 bg-zinc-900 text-white w-36 focus:outline-none focus:border-sky-400"
            />
            <button onClick={commit} className="text-xs px-1.5 py-0.5 rounded bg-sky-800 text-sky-200 hover:bg-sky-700 border border-sky-600">✓</button>
            <button onClick={cancel} className="text-xs px-1.5 py-0.5 rounded bg-zinc-800 text-zinc-400 hover:bg-zinc-700 border border-zinc-600">✕</button>
        </span>
    );
}

function ConfigGroup({
    group,
    keys,
    fullKeys,
    onSave,
}: {
    group: string;
    keys: Record<string, unknown>;
    fullKeys: Record<string, unknown>;
    onSave: (path: string, value: unknown) => void;
}) {
    const [open, setOpen] = useState(true);
    const entries = Object.entries(keys);

    return (
        <div className="border border-zinc-800 rounded-lg overflow-hidden">
            <button
                onClick={() => setOpen(o => !o)}
                className="w-full flex items-center justify-between px-4 py-2.5 bg-zinc-900 hover:bg-zinc-800 transition-colors text-left"
            >
                <span className="font-mono text-xs text-zinc-400 tracking-widest uppercase">{group}</span>
                <span className="text-zinc-600 text-xs">{open ? "▴" : "▾"} {entries.length} key{entries.length !== 1 ? "s" : ""}</span>
            </button>
            {open && (
                <div className="divide-y divide-zinc-800/60">
                    {entries.map(([leaf, value]) => {
                        const fullPath = group === "(root)" ? leaf : `${group}.${leaf}`;
                        return (
                            <div key={leaf} className="flex items-center justify-between px-4 py-2 gap-4 hover:bg-zinc-900/40">
                                <span className="font-mono text-sm text-zinc-300 min-w-0 truncate" title={fullPath}>
                                    {leaf}
                                </span>
                                <ConfigValue
                                    path={fullPath}
                                    leafKey={leaf}
                                    value={value}
                                    onSave={onSave}
                                />
                            </div>
                        );
                    })}
                </div>
            )}
        </div>
    );
}

// ── Page ─────────────────────────────────────────────────────────────────────

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
                    String(value).toLowerCase().includes(q)
                )
            );
            if (Object.keys(filtered).length > 0) result[group] = filtered;
        }
        return result;
    }, [groups, search]);

    const totalKeys = Object.keys(configKeys).length;
    const filteredCount = Object.values(filteredGroups).reduce((n, g) => n + Object.keys(g).length, 0);

    // ── Not connected ────────────────────────────────────────────────────
    if (!connected) {
        return (
            <div className="flex items-center justify-center h-64 text-zinc-600 text-sm font-mono">
                not connected
            </div>
        );
    }

    if (!configLoaded) {
        return (
            <div className="flex items-center justify-center h-64 text-zinc-600 text-sm font-mono gap-2">
                <span className="inline-block w-2 h-2 rounded-full bg-zinc-600 animate-pulse" />
                waiting for config…
            </div>
        );
    }

    return (
        <div className="max-w-3xl mx-auto px-4 py-8 space-y-6">

            {/* ── Header ── */}
            <div className="flex items-start justify-between gap-4">
                <div>
                    <h1 className="text-white text-lg font-mono font-medium tracking-tight">Configuration</h1>
                    <p className="text-zinc-500 text-xs font-mono mt-0.5">
                        {totalKeys} key{totalKeys !== 1 ? "s" : ""}
                        {search && ` · ${filteredCount} matching`}
                    </p>
                </div>
                <button
                    onClick={requestSnapshot}
                    className="text-xs font-mono px-3 py-1.5 rounded border border-zinc-700 text-zinc-400 hover:text-white hover:border-zinc-500 transition-colors"
                >
                    ↺ refresh
                </button>
            </div>

            {/* ── Ack toast ── */}
            {lastAck && (
                <div className={`text-xs font-mono px-3 py-2 rounded border ${lastAck.success
                        ? "bg-emerald-950 border-emerald-800 text-emerald-400"
                        : "bg-red-950 border-red-800 text-red-400"
                    }`}>
                    {lastAck.success ? "✓" : "✕"} {lastAck.message ?? (lastAck.success ? "ok" : "error")}
                </div>
            )}

            {/* ── Preset bar ── */}
            <div className="border border-zinc-800 rounded-lg p-4 space-y-3">
                <div className="flex items-center justify-between">
                    <span className="text-xs font-mono text-zinc-400 uppercase tracking-widest">Preset</span>
                    <span className="text-xs font-mono text-sky-400 bg-sky-950 border border-sky-800 px-2 py-0.5 rounded">
                        {currentPreset || "—"}
                    </span>
                </div>

                {presets.length > 0 && (
                    <div className="flex flex-wrap gap-2">
                        {presets.map((preset: any) => (
                            <button
                                key={preset}
                                onClick={() => loadPreset(preset)}
                                className={`text-xs font-mono px-2.5 py-1 rounded border transition-colors ${preset === currentPreset
                                        ? "border-sky-700 bg-sky-950 text-sky-300"
                                        : "border-zinc-700 bg-zinc-900 text-zinc-400 hover:border-zinc-500 hover:text-white"
                                    }`}
                            >
                                {preset}
                            </button>
                        ))}
                    </div>
                )}

                {/* Save as */}
                {savingAs ? (
                    <div className="flex items-center gap-2">
                        <input
                            autoFocus
                            value={saveInput}
                            onChange={e => setSaveInput(e.target.value)}
                            onKeyDown={e => {
                                if (e.key === "Enter" && saveInput.trim()) {
                                    savePreset(saveInput.trim());
                                    setSavingAs(false);
                                    setSaveInput("");
                                }
                                if (e.key === "Escape") setSavingAs(false);
                            }}
                            placeholder="filename.json"
                            className="font-mono text-xs px-2 py-1 rounded border border-zinc-600 bg-zinc-900 text-white flex-1 focus:outline-none focus:border-sky-500"
                        />
                        <button
                            onClick={() => { if (saveInput.trim()) { savePreset(saveInput.trim()); setSavingAs(false); setSaveInput(""); } }}
                            className="text-xs px-2 py-1 rounded bg-sky-800 text-sky-200 hover:bg-sky-700 border border-sky-600 font-mono"
                        >
                            save
                        </button>
                        <button
                            onClick={() => setSavingAs(false)}
                            className="text-xs px-2 py-1 rounded bg-zinc-800 text-zinc-400 hover:bg-zinc-700 border border-zinc-600 font-mono"
                        >
                            cancel
                        </button>
                    </div>
                ) : (
                    <button
                        onClick={() => { setSaveInput(currentPreset || ""); setSavingAs(true); }}
                        className="text-xs font-mono text-zinc-500 hover:text-zinc-300 transition-colors"
                    >
                        + save as new preset…
                    </button>
                )}
            </div>

            {/* ── Search ── */}
            <input
                value={search}
                onChange={e => setSearch(e.target.value)}
                placeholder="filter keys…"
                className="w-full font-mono text-sm px-3 py-2 rounded-lg border border-zinc-800 bg-zinc-900 text-white placeholder-zinc-600 focus:outline-none focus:border-zinc-600"
            />

            {/* ── Key groups ── */}
            <div className="space-y-2">
                {Object.entries(filteredGroups).map(([group, keys]) => (
                    <ConfigGroup
                        key={group}
                        group={group}
                        keys={keys}
                        fullKeys={configKeys}
                        onSave={setConfigKey}
                    />
                ))}
                {Object.keys(filteredGroups).length === 0 && (
                    <p className="text-center text-zinc-600 text-sm font-mono py-8">no keys match</p>
                )}
            </div>
        </div>
    );
}