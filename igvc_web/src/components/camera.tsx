"use client";
import { useRobotStore } from "@/lib/robot";

export default function Camera(props: CameraProps) {
  const path = useRobotStore((s) => s.path);
  const newPath = path.replace("ws://", "").split(":")[0];

  return (
    <div className="rounded-xl border border-neutral-700 bg-black overflow-hidden relative inline-block">
      <div className="absolute top-2 left-2 bg-crimson-glow text-white text-xs font-mono px-1 rounded-lg z-10">
        {props.id}
      </div>
      <img
        src={`http://${newPath}:8001/${props.id}`}
        className="block"
      />
    </div>
  );
}