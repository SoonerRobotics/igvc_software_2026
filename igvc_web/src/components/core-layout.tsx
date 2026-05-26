"use client";

import { useEffect } from "react";
import { SettingsIcon, LayoutDashboard, LineStyle, MessageSquare } from "lucide-react";
import { useRobotStore } from "@/lib/robot";
import LoadingOverlay from "./loading";
import { DropdownMenu, DropdownMenuTrigger, DropdownMenuGroup, DropdownMenuContent } from "./ui/dropdown";
import { Switch } from "./ui/switch";
import { Label } from "./ui/label";

const NAV_LINKS = [
    { label: "Dashboard", path: "/", icon: <LayoutDashboard size={14} /> },
    { label: "Configuration", path: "/config", icon: <SettingsIcon size={14} /> },
    { label: "Map", path: "/map", icon: <SettingsIcon size={14} /> },
    // { label: "Vision", path: "/vision", icon: <Camera size={14} /> },
    // { label: "Performance", path: "/performance", icon: <Clock size={14} /> },
    { label: "Subsystems", path: "/subsystems", icon: <LineStyle size={14} /> },
    { label: "Logs", path: "/logs", icon: <MessageSquare size={14} /> },
    // { label: "Power", path: "/electrical", icon: <BatteryFull size={14} /> },
    // { label: "Canbus", path: "/canbus", icon: <BatteryFull size={14} /> },
];

const STATE_OPTIONS = [
    { label: "Disabled", value: 0 },
    { label: "Manual", value: 1 },
    { label: "Autonomous", value: 2 },
];

const MISSION_OPTIONS = [
    { label: "Autonav", value: 0 },
    { label: "Self Drive", value: 1 },
];


export default function CoreLayout(props: { children?: React.ReactNode }) {
    const connect = useRobotStore((s) => s.connect);
    const connected = useRobotStore((s) => s.connected);

    const mobility = useRobotStore((s) => s.state.mobility);
    const mission = useRobotStore((s) => s.state.mission);
    const mode = useRobotStore((s) => s.state.mode);
    const setMobility = useRobotStore((s) => s.setMobility);
    const setMission = useRobotStore((s) => s.setMission);
    const setMode = useRobotStore((s) => s.setMode);

    useEffect(() => {
        connect();
    }, []);

    if (!connected) {
        return <LoadingOverlay />;
    }

    return (
        <div className="flex h-full flex-row bg-dark">
            <aside className="h-full w-48 flex-col bg-surface text-neutral-100 border-r border-neutral-700">
                <div className="flex items-center justify-center h-18 border-b border-neutral-700">
                    <span className="text-md text-center font-semibold tracking-widest uppercase">
                        Suspended Disbelief
                    </span>
                </div>

                <div className="flex flex-col gap-2 px-2 py-4">
                    {NAV_LINKS.map((link) => (
                        <a
                            key={link.path}
                            href={link.path}
                            className="w-full flex items-center space-x-2 rounded-md px-3 py-2 text-sm font-medium hover:bg-neutral-700"
                        >
                            {link.icon}
                            <span>{link.label}</span>
                        </a>
                    ))}
                </div>

                <div className="border-t border-neutral-700" />

                <div className="flex flex-col gap-2 px-2 py-4 mt-auto">
                    {/* dropdown each for state/mission, toggle for mobility - state is disabled (0), manual (1), and autonomous (2) : mission is 0 for autonav, 1 for self drive*/}
                    <div className="flex items-center gap-2 bg-neutral-800 rounded-md px-3 py-2 justify-between">
                        <Label htmlFor="r1">Mobility</Label>
                        <Switch id="r1" checked={mobility} onCheckedChange={(v) => setMobility(v)} />
                    </div>

                    <DropdownMenu>
                        <DropdownMenuTrigger className="w-full">
                            <div className="w-full flex items-center justify-between rounded-md bg-neutral-800 px-3 py-2 text-sm font-medium hover:bg-neutral-700">
                                <span>Mission</span>
                                <span>{MISSION_OPTIONS.find((o) => o.value === mission)?.label}</span>
                            </div>
                        </DropdownMenuTrigger>
                        <DropdownMenuContent className="bg-neutral-800 border border-neutral-700">
                            <DropdownMenuGroup>
                                {MISSION_OPTIONS.map((option) => (
                                    <button
                                        key={option.value}
                                        onClick={() => setMission(option.value)}
                                        className="w-full text-left px-4 py-2 hover:bg-neutral-700"
                                    >
                                        {option.label}
                                    </button>
                                ))}
                            </DropdownMenuGroup>
                        </DropdownMenuContent>
                    </DropdownMenu>

                    <DropdownMenu>
                        <DropdownMenuTrigger className="w-full">
                            <div className="w-full flex items-center justify-between rounded-md bg-neutral-800 px-3 py-2 text-sm font-medium hover:bg-neutral-700">
                                <span>Mode</span>
                                <span>{STATE_OPTIONS.find((o) => o.value === mode)?.label}</span>
                            </div>
                        </DropdownMenuTrigger>
                        <DropdownMenuContent className="bg-neutral-800 border border-neutral-700">
                            <DropdownMenuGroup>
                                {STATE_OPTIONS.map((option) => (
                                    <button
                                        key={option.value}
                                        onClick={() => setMode(option.value)}
                                        className="w-full text-left px-4 py-2 hover:bg-neutral-700"
                                    >
                                        {option.label}
                                    </button>
                                ))}
                            </DropdownMenuGroup>
                        </DropdownMenuContent>
                    </DropdownMenu>
                </div>
            </aside>

            <div className="flex flex-1 flex-col overflow-hidden">
                {/* Page Content */}
                <main className="flex-1 overflow-auto p-8">
                    {props.children}
                </main>
            </div>
        </div>
    );
}