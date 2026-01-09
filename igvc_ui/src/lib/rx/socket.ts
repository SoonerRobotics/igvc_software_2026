import { webSocket } from "rxjs/webSocket";
import { WS_URL } from "../constants";

export const socket$ = webSocket({
    url: WS_URL,

    binaryType: "arraybuffer",
    deserializer: msg => msg.data as ArrayBuffer,
    serializer: msg => msg,
})