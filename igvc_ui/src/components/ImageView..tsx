import { Observable } from "rxjs";
import { useObservable } from "../hooks/useObservable";
import { useEffect } from "react";

export function ImageView({ stream$ }: { stream$: Observable<Blob> }) {
    const blob = useObservable(stream$);

    useEffect(() => {
        return () => {
            URL.revokeObjectURL(blob ? URL.createObjectURL(blob) : "");
        };
    }, [blob]);
    
    return (<img src={blob ? URL.createObjectURL(blob) : ""} alt="Image Stream" />);
}