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
package ai.djl.paddlepaddle.engine;

import ai.djl.Device;
import ai.djl.Model;
import ai.djl.engine.Engine;
import ai.djl.ndarray.NDManager;
import ai.djl.paddlepaddle.jni.LibUtils;

import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

/**
 * The {@code PpEngine} is an implementation of the {@link Engine} based on the <a
 * href="https://github.com/PaddlePaddle/Paddle/">PaddlePaddle</a>.
 *
 * <p>To get an instance of the {@code PpEngine} when it is not the default Engine, call {@link
 * Engine#getEngine(String)} with the Engine name "PaddlePaddle".
 */
public final class PpEngine extends Engine {

    public static final String ENGINE_NAME = "PaddlePaddle";
    static final int RANK = 10;

    private Engine alternativeEngine;
    private boolean initialized;

    private PpEngine() {}

    static Engine newInstance() {
        LibUtils.loadLibrary();
        return new PpEngine();
    }

    /** {@inheritDoc} */
    @Override
    public Engine getAlternativeEngine() {
        if (!initialized && !Boolean.getBoolean("ai.djl.paddlepaddle.disable_alternative")) {
            Engine engine = Engine.getInstance();
            if (engine.getRank() < getRank()) {
                // alternativeEngine should not have the same rank as PaddlePaddle
                alternativeEngine = engine;
            }
            initialized = true;
        }
        return alternativeEngine;
    }

    /** {@inheritDoc} */
    @Override
    public String getEngineName() {
        return ENGINE_NAME;
    }

    /** {@inheritDoc} */
    @Override
    public int getRank() {
        return RANK;
    }

    /** {@inheritDoc} */
    @Override
    public String getVersion() {
        try (InputStream is =
                PpEngine.class.getResourceAsStream("/paddlepaddle-engine.properties")) {
            Properties prop = new Properties();
            prop.load(is);
            return prop.getProperty("paddlepaddle_version");
        } catch (IOException e) {
            throw new AssertionError("Failed to load paddlapaddle-engine.properties", e);
        }
    }

    /** {@inheritDoc} */
    @Override
    public boolean hasCapability(String capability) {
        // Default device is always CPU
        return false;
    }

    /** {@inheritDoc} */
    @Override
    public Model newModel(String name, Device device) {
        return new PpModel(name, device, newBaseManager(device));
    }

    /** {@inheritDoc} */
    @Override
    public NDManager newBaseManager() {
        return newBaseManager(null);
    }

    /** {@inheritDoc} */
    @Override
    public NDManager newBaseManager(Device device) {
        return PpNDManager.getSystemManager().newSubManager(device);
    }
}
