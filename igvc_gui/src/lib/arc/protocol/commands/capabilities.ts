import { makeMessageWrapperBB, MessageType } from "../types";
import { constructCapabilityRequest } from "../constructors";
import decoders from "../decoders";
import { sendMessage, subscribe } from "$lib/arc/socket";
import { ArcCapability } from "$lib/arc/messages/messages/arc/arc-capability";

// TODO: maybe switch to writable store for capabilities
let currentCapabilities = {
    visionMask: 0,
    telemetryMask: 0,
    miscMask: 0
};

// TODO: Will this cause issues on rerender?
subscribe<ArcCapability>(MessageType.CapabilityAck, decoders.capability_ack, msg => {
    currentCapabilities = {
        visionMask: msg.visionCapabilities(),
        telemetryMask: msg.telemetryCapabilities(),
        miscMask: msg.miscCapabilities()
    };
})

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
    const wrapper = makeMessageWrapperBB(MessageType.CapabilityReq, capability.bb!);
    sendMessage(wrapper);
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

export function removeVisionCapability(visionMask: number) {
    currentCapabilities.visionMask &= ~visionMask;
    requestCapabilities(currentCapabilities.visionMask, currentCapabilities.telemetryMask, currentCapabilities.miscMask);
}