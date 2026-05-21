export type CameraProps = {
    id: string;
}

export default function Camera(props: CameraProps)
{
    return (
        <div className="rounded-xl border border-neutral-700 bg-black aspect-video overflow-hidden relative">
            <div className="absolute top-2 left-2 bg-crimson-glow text-white text-xs font-mono px-1 rounded-lg">
                {props.id}
            </div>

            <img src={`http://localhost:8001/${props.id}`} className="w-full h-full object-cover" />
        </div>
    )
}