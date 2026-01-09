import { useEffect, useState } from "react";
import { Observable } from "rxjs";

export function useObservable(observable$: Observable<any>, initialValue: any = null) {
    const [value, setValue] = useState(initialValue);

    useEffect(() => {
        const sub = observable$.subscribe(setValue);
        return () => sub.unsubscribe();
    }, [observable$]);

    return value;
}
