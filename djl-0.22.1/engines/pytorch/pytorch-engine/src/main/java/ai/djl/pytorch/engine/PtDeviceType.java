/*
 * Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You may not use this file except in compliance
 * with the License. A copy of the License is located at
 *
 * http://aws.amazon.com/apache2.0/
 *
 * or in the "license" file accompanying this file. This file is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions
 * and limitations under the License.
 */
package ai.djl.pytorch.engine;

import ai.djl.Device;

/** DeviceType is the PyTorch equivalent of the types in {@link Device}. */
public final class PtDeviceType {

    private PtDeviceType() {}

    /**
     * Converts a {@link Device} to the corresponding PyTorch device number.
     *
     * @param device the java {@link Device}
     * @return the PyTorch device number
     */
    public static int toDeviceType(Device device) {
        String deviceType = device.getDeviceType();

        if (Device.Type.CPU.equals(deviceType)) {
            return 0;
        } else if (Device.Type.GPU.equals(deviceType)) {
            return 1;
        } else if ("mps".equals(deviceType)) {
            return 13;
        } else {
            throw new IllegalArgumentException("Unsupported device: " + device);
        }
    }

    /**
     * Converts from an PyTorch device number to {@link Device}.
     *
     * @param deviceType the PyTorch device number
     * @return the corresponding {@link Device}
     */
    public static String fromDeviceType(int deviceType) {
        switch (deviceType) {
            case 0:
                return Device.Type.CPU;
            case 1:
                return Device.Type.GPU;
            case 13:
                return "mps";
            default:
                throw new IllegalArgumentException("Unsupported deviceType: " + deviceType);
        }
    }
}
