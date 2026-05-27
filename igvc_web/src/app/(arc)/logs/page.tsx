"use client";

import { Badge } from "@/components/ui/badge";
import { useRobotStore } from "@/lib/robot";

type LogLevel = "DEBUG" | "INFO" | "WARN" | "ERROR" | "FATAL" | "TRACE";

const levelVariant: Record<LogLevel, any> = {
    TRACE: "secondary",
    DEBUG: "secondary",
    INFO: "info",
    WARN: "warning",
    ERROR: "destructive",
    FATAL: "destructive",
};

function logLevelFromNumber(level: number): LogLevel {
    switch (level) {
        case 0:
            return "TRACE";
        case 1:
            return "DEBUG";
        case 2:
            return "INFO";
        case 3:
            return "WARN";
        case 4:
            return "ERROR";
        case 5:
            return "FATAL";
        default:
            return "INFO";
    }
}

export default function LogsPage() {
    const logs = useRobotStore((state) => state.logs.reverse());

    return (
        <div className="flex flex-col">
            <h1 className="text-2xl font-semibold text-white">Logs</h1>

            <div className="mt-4 rounded-lg border border-white/10 overflow-hidden">
                <table className="w-full text-sm text-white">
                    <thead>
                        <tr className="border-b border-white/10 bg-white/5 text-xs text-white/50 uppercase tracking-wider">
                            <th className="px-4 py-3 text-left">Namespace</th>
                            <th className="px-4 py-3 text-left">Level</th>
                            <th className="px-4 py-3 text-left">Message</th>
                        </tr>
                    </thead>
                    <tbody>
                        {logs.map((log) => (
                            // create hash of log message + timestamp to use as key
                            <tr
                                key={`${log.message}-${new Date().getTime()}`}
                                className="border-b border-white/5 hover:bg-white/5 transition-colors"
                            >
                                <td className="px-4 py-3 font-mono text-xs text-white/50">
                                    {log.cat}
                                </td>
                                <td className="px-4 py-3">
                                    <Badge variant={levelVariant[logLevelFromNumber(log.level)]}>
                                        {logLevelFromNumber(log.level)}
                                    </Badge>
                                </td>
                                <td className="px-4 py-3 text-white/80">{log.message}</td>
                            </tr>
                        ))}
                    </tbody>
                </table>
            </div>
        </div>
    );
}