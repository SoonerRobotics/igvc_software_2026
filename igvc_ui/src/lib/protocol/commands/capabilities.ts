import { capabilityMessages$ } from "../../rx/listeners/capabilities";
import { socket$ } from "../../rx/socket";
import { makeMessageWrapperBB, MessageType } from "../../types";
import { constructCapabilityRequest } from "../constructors";
import { MessageWriter } from "../message_writer";

let currentCapabilities = {
    visionMask: 0,
    telemetryMask: 0,
    miscMask: 0
};

// checkers

export const hasVisionCapability = (visionMask: number): boolean => {
    return (currentCapabilities.visionMask & visionMask) === visionMask;
}

export const hasTelemetryCapability = (telemetryMask: number): boolean => {
    return (currentCapabilities.telemetryMask & telemetryMask) === telemetryMask;
}

export const hasMiscCapability = (miscMask: number): boolean => {
    return (currentCapabilities.miscMask & miscMask) === miscMask;
}

// requesters

export const requestCapabilities = (visionMask: number, telemetryMask: number, miscMask: number) => {
    const capability = constructCapabilityRequest(visionMask, telemetryMask, miscMask);
    const wrapper = makeMessageWrapperBB(MessageType.CapabiltiyReq, capability.bb!);

    console.log("Requesting capabilities:", {
        visionMask,
        telemetryMask,
        miscMask
    });
    const frame = MessageWriter.writeWrapper(MessageType.CapabiltiyReq, wrapper);
    socket$.next(frame.buffer as ArrayBuffer);
}

export function addVisionCapability(visionMask: number) {
    currentCapabilities.visionMask |= visionMask;
    requestCapabilities(currentCapabilities.visionMask, currentCapabilities.telemetryMask, currentCapabilities.miscMask);
}

export function addTelemetryCapability(telemetryMask: number) {
    currentCapabilities.telemetryMask |= telemetryMask;
    requestCapabilities(currentCapabilities.visionMask, currentCapabilities.telemetryMask, currentCapabilities.miscMask);
}

export function addMiscCapability(miscMask: number) {
    currentCapabilities.miscMask |= miscMask;
    requestCapabilities(currentCapabilities.visionMask, currentCapabilities.telemetryMask, currentCapabilities.miscMask);
}

// listener

capabilityMessages$.subscribe(msg => {
    currentCapabilities = {
        visionMask: msg.visionCapabilities(),
        telemetryMask: msg.telemetryCapabilities(),
        miscMask: msg.miscCapabilities()
    };
    console.log("Updated capabilities:", currentCapabilities);
});