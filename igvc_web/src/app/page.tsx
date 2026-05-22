"use client";

import Camera from "@/components/camera";
import ConfigurationModal from "@/components/configuration";
import { Button } from "@mantine/core";
import { useState } from "react";

export default function Home() {
    return (
        <div className="flex flex-col">
            <div className="flex flex-row items-center max-w-full">
                <Camera id="left" />
                <Camera id="right" />
                {/* <Camera id="combined_filtered" /> */}
                <Camera id="debug_feelers" />
            </div>
        </div>
    );
}
