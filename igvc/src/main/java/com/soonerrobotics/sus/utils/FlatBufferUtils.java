package com.soonerrobotics.sus.utils;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import com.soonerrobotics.Result;
import com.soonerrobotics.flatbuffers.ImageFrame;
import com.soonerrobotics.flatbuffers.arc.ArcLog;

public class FlatBufferUtils {
    public enum FlatBufferType {
        IMAGE_FRAME((byte) 0x01),
        ARC_LOG((byte) 0x02);

        private final byte value;

        FlatBufferType(byte value) {
            this.value = value;
        }

        public byte getValue() {
            return value;
        }

        public static FlatBufferType fromByte(byte b) {
            for (FlatBufferType type : FlatBufferType.values()) {
                if (type.value == b) {
                    return type;
                }
            }

            return null;
        }
    }

    public static class FlatBufferWrapper {
        public FlatBufferType messageType;
        public int payloadLength;
        public byte[] payload;

        public FlatBufferWrapper(FlatBufferType messageType, int payloadLength, byte[] payload) {
            if (payload.length != payloadLength) {
                throw new IllegalArgumentException("Payload length does not match actual payload size");
            }

            this.messageType = messageType;
            this.payloadLength = payloadLength;
            this.payload = payload;
        }

        public static Result<FlatBufferWrapper> fromByteArray(byte[] buffer) {
            try {
                int offset = 0;

                byte messageType = buffer[offset++];
                int payloadLength = ((buffer[offset++] & 0xff) << 24) |
                        ((buffer[offset++] & 0xff) << 16) |
                        ((buffer[offset++] & 0xff) << 8) |
                        (buffer[offset++] & 0xff);

                if (payloadLength < 0 || offset + payloadLength > buffer.length) {
                    throw new IOException("Invalid payload length");
                }

                byte[] payload = new byte[payloadLength];
                System.arraycopy(buffer, offset, payload, 0, payloadLength);

                return Result.success(new FlatBufferWrapper(
                        FlatBufferType.fromByte(messageType),
                        payloadLength,
                        payload));
            } catch (Exception e) {
                return Result.fail(e);
            }
        }

        /**
         * Create a FlatBufferWrapper
         * 
         * @param type FlatBuffer type
         */
        public static Result<FlatBufferWrapper> create(FlatBufferType type, byte[] payload) {
            return Result.success(new FlatBufferWrapper(type, payload.length, payload));
        }

        /**
         * Convert to byte array with specified byte order
         * 
         * @param order Byte order to use
         * @return Byte array representation
         */
        public byte[] toByteArray(ByteOrder order) {
            // Total length: message type (1) + payload length (4) + payload
            int totalLength = 1 + 4 + payloadLength;
            byte[] byteArray = new byte[totalLength];

            int index = 0;
            byteArray[index++] = messageType.getValue();

            if (order == ByteOrder.BIG_ENDIAN) {
                byteArray[index++] = (byte) ((payloadLength >> 24) & 0xFF);
                byteArray[index++] = (byte) ((payloadLength >> 16) & 0xFF);
                byteArray[index++] = (byte) ((payloadLength >> 8) & 0xFF);
                byteArray[index++] = (byte) (payloadLength & 0xFF);
            } else {
                byteArray[index++] = (byte) (payloadLength & 0xFF);
                byteArray[index++] = (byte) ((payloadLength >> 8) & 0xFF);
                byteArray[index++] = (byte) ((payloadLength >> 16) & 0xFF);
                byteArray[index++] = (byte) ((payloadLength >> 24) & 0xFF);
            }

            System.arraycopy(payload, 0, byteArray, index, payloadLength);

            return byteArray;
        }

        /**
         * Convert to byte array using native byte order
         * 
         * @return Byte array representation
         */
        public byte[] toByteArray() {
            return toByteArray(ByteOrder.nativeOrder());
        }

        /**
         * Get the payload
         * 
         * @return Payload byte array
         */
        public byte[] getPayload() {
            return payload;
        }
    }

    public static class FlatBufferConverter {
        private static void checkType(FlatBufferWrapper wrapper, FlatBufferType expectedType) {
            if (wrapper.messageType != expectedType) {
                throw new IllegalArgumentException("FlatBufferWrapper is not of type " + expectedType);
            }
        }

        public static ImageFrame asImageFrame(FlatBufferWrapper wrapper) {
            checkType(wrapper, FlatBufferType.IMAGE_FRAME);
            return ImageFrame.getRootAsImageFrame(ByteBuffer.wrap(wrapper.payload));
        }

        public static ArcLog asArcLog(FlatBufferWrapper wrapper) {
            checkType(wrapper, FlatBufferType.ARC_LOG);
            return ArcLog.getRootAsArcLog(ByteBuffer.wrap(wrapper.payload));
        }
    }
}
