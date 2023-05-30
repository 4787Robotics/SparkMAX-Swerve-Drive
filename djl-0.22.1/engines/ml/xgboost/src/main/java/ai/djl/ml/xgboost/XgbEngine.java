/*
 * Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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
package ai.djl.ml.xgboost;

import ai.djl.Device;
import ai.djl.Model;
import ai.djl.engine.Engine;
import ai.djl.engine.StandardCapabilities;
import ai.djl.ndarray.NDManager;

import ml.dmlc.xgboost4j.java.JniUtils;

import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

/**
 * The {@code XgbEngine} is an implementation of the {@link Engine} based on the <a
 * href="https://github.com/dmlc/xgboost">XGBoost</a>.
 *
 * <p>To get an instance of the {@code XgbEngine} when it is not the default Engine, call {@link
 * Engine#getEngine(String)} with the Engine name "XGBoost".
 */
public final class XgbEngine extends Engine {

    public static final String ENGINE_NAME = "XGBoost";
    static final int RANK = 10;

    private Engine alternativeEngine;
    private boolean initialized;

    private XgbEngine() {}

    static Engine newInstance() {
        JniUtils.checkCall(0); // Load the native
        return new XgbEngine();
    }

    /** {@inheritDoc} */
    @Override
    public Engine getAlternativeEngine() {
        if (!initialized && !Boolean.getBoolean("ai.djl.xgboost.disable_alternative")) {
            Engine engine = Engine.getInstance();
            if (engine.getRank() < getRank()) {
                // alternativeEngine should not have the same rank as OnnxRuntime
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
                XgbEngine.class.getResourceAsStream("/xgboost4j-version.properties")) {
            Properties prop = new Properties();
            prop.load(is);
            return prop.getProperty("version");
        } catch (IOException e) {
            throw new AssertionError("Failed to load xgboost4j-version.properties", e);
        }
    }

    /** {@inheritDoc} */
    @Override
    public boolean hasCapability(String capability) {
        if (StandardCapabilities.CUDA.equals(capability)) {
            try {
                Class.forName("ml.dmlc.xgboost4j.gpu.java.CudfColumn");
                return true;
            } catch (ClassNotFoundException ignore) {
                return false;
            }
        }
        return false;
    }

    /** {@inheritDoc} */
    @Override
    public Model newModel(String name, Device device) {
        return new XgbModel(name, newBaseManager(device));
    }

    /** {@inheritDoc} */
    @Override
    public NDManager newBaseManager() {
        return newBaseManager(null);
    }

    /** {@inheritDoc} */
    @Override
    public NDManager newBaseManager(Device device) {
        return XgbNDManager.getSystemManager().newSubManager(device);
    }

    /** {@inheritDoc} */
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder(200);
        sb.append(getEngineName()).append(':').append(getVersion()).append(", ");
        sb.append(getEngineName()).append(':').append(getVersion()).append(", capabilities: [\n");
        if (hasCapability(StandardCapabilities.CUDA)) {
            sb.append("\t").append(StandardCapabilities.CUDA).append(",\n"); // NOPMD
        }
        return sb.toString();
    }
}
