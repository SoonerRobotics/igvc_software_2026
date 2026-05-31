"use client";

import Camera from "@/components/camera";
import ConfigurationModal from "@/components/configuration";
import { useRobotStore } from "@/lib/robot";
import { useState } from "react";

export default function Home() {
    const vn = useRobotStore((s) => s.vectornav);
    const voltage = useRobotStore((s) => s.voltage);
    const current = useRobotStore((s) => s.current);
    const ws = useRobotStore((s) => s.waypointState);
    const calibrate = useRobotStore((s) => s.calibrateHsvThreshold);

    return (
        <div className="flex flex-col">
            {/* Various Properties (e.g. latitude/longitude, yaw) */}
            <div>
                <div className="flex flex-row items-center max-w-full">
                    <div className="flex flex-col">
                        <span className="text-sm text-white">Latitude</span>
                        <span className="text-lg text-white/80">{vn?.latitude().toFixed(6) ?? "N/A"}</span>
                    </div>
                    <div className="flex flex-col ml-4">
                        <span className="text-sm text-white">Longitude</span>
                        <span className="text-lg text-white/80">{vn?.longitude().toFixed(6) ?? "N/A"}</span>
                    </div>
                    <div className="flex flex-col ml-4">
                        <span className="text-sm text-white">Num Sats</span>
                        <span className="text-lg text-white/80">{vn?.numSatellites() ?? "N/A"}</span>
                    </div>
                    <div className="flex flex-col ml-4">
                        <span className="text-sm text-white">Yaw (Coord Based)</span>
                        <span className="text-lg text-white/80">{ws?.bearingDegrees?.toFixed(2) ?? "N/A"}°</span>
                    </div>
                    <div className="flex flex-col ml-4">
                        <span className="text-sm text-white">Yaw (VN Based)</span>
                        <span className="text-lg text-white/80">{vn?.yaw()?.toFixed(2) ?? "N/A"}°</span>
                    </div>
                </div>

                <button className="px-4 py-2 bg-blue-500 text-white rounded mt-4" onClick={() => calibrate()}>
                    Calibrate HSV Threshold
                </button>
            </div>

            <div className="flex flex-row items-center max-w-full">
                <Camera id="center" />
                {/* <Camera id="debug_feelers" /> */}
            </div>

            <div className="flex flex-row items-center max-w-full">
                <Camera id="combined_debug" />
                <Camera id="combined_filtered" />
                <Camera id="combined_inflated" />
                {/* <Camera id="debug_feelers" /> */}
            </div>

            <div className="flex flex-row items-center max-w-full">
                <Camera id="nav_astar_debug" />
                <Camera id="nav_path_overlay" />
                {/* <Camera id="debug_feelers" /> */}
            </div>

            <div className="flex flex-row items-center max-w-full">
                <Camera id="zed" />
                <Camera id="yolo" />
                {/* <Camera id="debug_feelers" /> */}
            </div>

            <div>
                <span className="text-sm text-white">Voltage</span>
                <span className="text-lg text-white/80">{voltage ? `${voltage.toFixed(2)} V` : "N/A"}</span>

                <span className="text-sm text-white ml-4">Current</span>
                <span className="text-lg text-white/80">{current ? `${current.toFixed(2)} A` : "N/A"}</span>
            </div>
        </div>
    );
}
