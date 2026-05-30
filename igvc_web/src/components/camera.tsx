import { useRobotStore } from "@/lib/robot";

export type CameraProps = {
    id: string;
}

export default function Camera(props: CameraProps)
{
    const path = useRobotStore((s) => s.path);

    // Extract ip from path (ws://x.x.x.x:port)
    const newPath = path.replace("ws://", "").split(":")[0];

    // Render at http://x.x.x.x:8081/id

    return (
        <div className="rounded-xl border border-neutral-700 bg-black aspect-video overflow-hidden relative">
            <div className="absolute top-2 left-2 bg-crimson-glow text-white text-xs font-mono px-1 rounded-lg">
                {props.id}
            </div>

            <img src={`http://${newPath}:8001/${props.id}`} className="w-fit h-fit object-cover" />
        </div>
    )
}