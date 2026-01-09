import { webSocket } from "rxjs/webSocket";
import { WS_URL } from "../constants";

export const socket$ = webSocket({
    url: WS_URL
})