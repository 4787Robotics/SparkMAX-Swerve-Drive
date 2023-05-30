/*
 * Copyright 2022 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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
package ai.djl.ml.lightgbm;

import ai.djl.BaseModel;
import ai.djl.Model;
import ai.djl.ml.lightgbm.jni.JniUtils;
import ai.djl.ndarray.NDManager;
import ai.djl.ndarray.types.DataType;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Map;

/** {@code LgbmModel} is the LightGBM implementation of {@link Model}. */
public class LgbmModel extends BaseModel {

    /**
     * Constructs a new Model on a given device.
     *
     * @param modelName the model name
     * @param manager the {@link NDManager} to holds the NDArray
     */
    LgbmModel(String modelName, NDManager manager) {
        super(modelName);
        dataType = DataType.FLOAT32;
        this.manager = manager;
        manager.setName("LgbmModel");
    }

    /** {@inheritDoc} */
    @Override
    public void load(Path modelPath, String prefix, Map<String, ?> options) throws IOException {
        setModelDir(modelPath);
        wasLoaded = true;
        if (block != null) {
            throw new UnsupportedOperationException("LightGBM does not support dynamic blocks");
        }
        Path modelFile = findModelFile(prefix);
        if (modelFile == null) {
            modelFile = findModelFile(modelDir.toFile().getName());
            if (modelFile == null) {
                throw new FileNotFoundException(".json file not found in: " + modelPath);
            }
        }
        block = JniUtils.loadModel((LgbmNDManager) manager, modelFile.toAbsolutePath().toString());
    }

    private Path findModelFile(String prefix) {
        if (Files.isRegularFile(modelDir)) {
            Path file = modelDir;
            modelDir = modelDir.getParent();
            String fileName = file.toFile().getName();
            if (fileName.endsWith(".txt")) {
                modelName = fileName.substring(0, fileName.length() - 4);
            } else {
                modelName = fileName;
            }
            return file;
        }
        if (prefix == null) {
            prefix = modelName;
        }
        Path modelFile = modelDir.resolve(prefix);
        if (Files.notExists(modelFile) || !Files.isRegularFile(modelFile)) {
            if (prefix.endsWith(".txt")) {
                return null;
            }
            modelFile = modelDir.resolve(prefix + ".txt");
            if (Files.notExists(modelFile) || !Files.isRegularFile(modelFile)) {
                return null;
            }
        }
        return modelFile;
    }

    /** {@inheritDoc} */
    @Override
    public void close() {
        if (block != null) {
            ((LgbmSymbolBlock) block).close();
            block = null;
        }
        super.close();
    }
}
