import { filter, map, sampleTime, share, shareReplay } from "rxjs";
import { messages$ } from "../messages";
import { MessageType } from "../../types";
import decoder from "../../protocol/decoders";

export const capabilityMessages$ = messages$.pipe(
    filter(m => m.type === MessageType.CapabilityAck),
    map(m => decoder.capability_ack(m)),
    share()
);