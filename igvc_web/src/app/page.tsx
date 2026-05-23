"use client";

import Camera from "@/components/camera";
import ConfigurationModal from "@/components/configuration";
import { useRobotStore } from "@/lib/robot";
import { useState } from "react";

export default function Home() {
    const vn = useRobotStore((s) => s.vectornav);

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
                        <span className="text-sm text-white">Yaw</span>
                        <span className="text-lg text-white/80">{vn?.yaw().toFixed(2) ?? "N/A"}°</span>
                    </div>
                </div>
            </div>

            <div className="flex flex-row items-center max-w-full">
                <Camera id="left" />
                <Camera id="right" />
                <Camera id="combined_filtered" />
                {/* <Camera id="debug_feelers" /> */}
            </div>

            <div className="flex flex-row items-center max-w-full">
                <Camera id="left_debug" />
                <Camera id="right_debug" />
                {/* <Camera id="debug_feelers" /> */}
            </div>
        </div>
    );
}
