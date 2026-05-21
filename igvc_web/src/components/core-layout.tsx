"use client";

import { useEffect, useState } from "react";
import { SettingsIcon, LayoutDashboard, Camera, Clock, LineStyle, MessageSquare, BatteryFull } from "lucide-react";
import { useRobotStore } from "@/lib/robot";

const NAV_LINKS = [
    { label: "Dashboard", path: "/", icon: <LayoutDashboard size={14} /> },
    { label: "Configuration", path: "/configuration", icon: <SettingsIcon size={14} /> },
    { label: "Vision", path: "/vision", icon: <Camera size={14} /> },
    { label: "Performance", path: "/performance", icon: <Clock size={14} /> },
    { label: "Subsystems", path: "/subsystems", icon: <LineStyle size={14} /> },
    { label: "Logs", path: "/logs", icon: <MessageSquare size={14} /> },
    { label: "Power", path: "/electrical", icon: <BatteryFull size={14} /> },
];

export default function CoreLayout(props: { children?: React.ReactNode }) {
    const connect = useRobotStore((s) => s.connect);

    useEffect(() => {
        connect("ws://localhost:8080");
    }, []);

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