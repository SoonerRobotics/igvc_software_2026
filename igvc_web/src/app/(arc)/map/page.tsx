"use client";

import { useRobotStore } from "@/lib/robot";
import { LOCATIONS } from "./locations";
import SatelliteMap from "./map";

export default function MapPage() {
    const vn = useRobotStore((s) => s.vectornav);

    return (
        <SatelliteMap
            location={LOCATIONS[1]}
            robot={
                vn ? {
                    lng: vn.longitude(),
                    lat: vn.latitude(),
                    heading: vn.yaw(),
                } : { lng: -97.442322, lat: 35.210544, heading: 0 }
            }
            waypoints={[
                { id: "wp1", lng: -97.442080, lat: 35.210590, radius: 2 , label: "A" },
                { id: "wp2", lng: -97.442312, lat: 35.210598, radius: 2, label: "B" },
                { id: "wp3", lng: -97.442314, lat: 35.210473, radius: 2, label: "C" },
                { id: "wp4", lng: -97.441919, lat: 35.210480, radius: 2, label: "D" },
            ]}
        />
    )
}