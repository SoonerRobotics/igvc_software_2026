import { Subject } from "rxjs";
import { MessageAccumulator } from "../protocol/message_accumulator";
import { Endianness, MessageWrapper } from "../types";
import { socket$ } from "./socket";

export const messages$ = new Subject<MessageWrapper>();
    
const accumulator = new MessageAccumulator(
  Endianness.Little,
  msg => messages$.next(msg)
);

socket$.subscribe(buffer => {
  accumulator.append(new Uint8Array(buffer));
});
