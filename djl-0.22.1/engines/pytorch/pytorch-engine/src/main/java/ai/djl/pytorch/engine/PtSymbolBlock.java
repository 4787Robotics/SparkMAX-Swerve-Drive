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

import ai.djl.MalformedModelException;
import ai.djl.ndarray.NDArray;
import ai.djl.ndarray.NDList;
import ai.djl.ndarray.NDManager;
import ai.djl.ndarray.types.DataType;
import ai.djl.ndarray.types.Shape;
import ai.djl.nn.AbstractSymbolBlock;
import ai.djl.nn.Parameter;
import ai.djl.nn.ParameterList;
import ai.djl.nn.SymbolBlock;
import ai.djl.pytorch.jni.IValue;
import ai.djl.pytorch.jni.IValueUtils;
import ai.djl.pytorch.jni.JniUtils;
import ai.djl.training.ParameterStore;
import ai.djl.util.PairList;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

/**
 * {@code PtSymbolBlock} is the PyTorch implementation of {@link SymbolBlock}.
 *
 * <p>You can create a {@code PtSymbolBlock} using {@link ai.djl.Model#load(java.nio.file.Path,
 * String)}.
 */
// TODO: Memory handling
public class PtSymbolBlock extends AbstractSymbolBlock implements AutoCloseable {

    private static final Logger logger = LoggerFactory.getLogger(PtSymbolBlock.class);

    private AtomicReference<Long> handle;
    private String uid;
    private PtNDManager manager;
    private boolean isTrain;
    private PairList<String, Shape> inputDescriptions;
    private PairList<String, Shape> outputDescriptions;
    private boolean first;
    private Map<String, Parameter> parameters;

    /**
     * Constructs a {@code PtSymbolBlock}.
     *
     * <p>You can create a {@code PtSymbolBlock} using {@link ai.djl.Model#load(java.nio.file.Path,
     * String)}.
     *
     * @param manager the manager to use for the block
     * @param handle the module handle
     */
    public PtSymbolBlock(PtNDManager manager, long handle) {
        this(manager);
        this.handle = new AtomicReference<>(handle);
        uid = String.valueOf(handle);
        manager.attachInternal(uid, this);
    }

    /**
     * Constructs an Empty {@code PtSymbolBlock}.
     *
     * @param manager the manager to use for the block
     */
    public PtSymbolBlock(PtNDManager manager) {
        this.manager = manager;
        // training mode is on by default
        isTrain = true;
        first = true;
    }

    /** {@inheritDoc} */
    @Override
    public void close() {
        Long pointer = handle.getAndSet(null);
        if (pointer != null) {
            JniUtils.deleteModule(pointer);
            manager.detachInternal(uid);
            manager = null;
        }
    }

    /**
     * Runs the forward of this PyTorch module.
     *
     * @param inputs the input {@link IValue}
     * @return the result {@link IValue}
     */
    public IValue forward(IValue... inputs) {
        return IValueUtils.forward(this, inputs);
    }

    /** {@inheritDoc} */
    @Override
    protected NDList forwardInternal(
            ParameterStore parameterStore,
            NDList inputs,
            boolean training,
            PairList<String, Object> params) {
        // TODO refactor the forward to not take ParameterStore
        if (isTrain != training) {
            isTrain = training;
            if (isTrain) {
                JniUtils.enableTrainingMode(this);
            } else {
                JniUtils.enableInferenceMode(this);
            }
        }

        if (System.getProperty("ai.djl.pytorch.graph_optimizer") != null) {
            /*
             * By default, graph_optimizer is enabled. But it requires a warm-up time in a few
             * inference calls. This optimizer setting is thread local, thus has to be disabled per
             * thread. User must programmatically call JniUtils.setGraphExecutorOptimize(false) if
             * he wants to disable graph optimizer per model.
             */
            boolean setOptimizer = Boolean.getBoolean("ai.djl.pytorch.graph_optimizer");
            JniUtils.setGraphExecutorOptimize(setOptimizer);
        }
        if (first) {
            synchronized (PtSymbolBlock.class) {
                if (first) {
                    inputDescriptions = new PairList<>();
                    outputDescriptions = new PairList<>();
                    for (NDArray array : inputs) {
                        inputDescriptions.add(array.getName(), array.getShape());
                    }
                    NDList outputs = IValueUtils.forward(this, inputs, training);
                    for (NDArray array : outputs) {
                        outputDescriptions.add(array.getName(), array.getShape());
                    }
                    first = false;
                    return outputs;
                }
            }
        }
        return IValueUtils.forward(this, inputs, training);
    }

    /** {@inheritDoc} */
    @Override
    public PairList<String, Shape> describeInput() {
        if (inputDescriptions == null) {
            logger.warn(
                    "Input shapes are unknown, please run predict or forward once"
                            + " and call describeInput again.");
        }
        return inputDescriptions;
    }

    /** {@inheritDoc} */
    @Override
    public ParameterList getDirectParameters() {
        if (parameters == null) {
            NDList params = JniUtils.moduleGetParams(this, manager);
            parameters = new LinkedHashMap<>(params.size());
            for (NDArray param : params) {
                parameters.put(
                        param.getName(),
                        Parameter.builder()
                                .setName(param.getName())
                                .setType(inferType(param.getName()))
                                .optArray(param)
                                .build());
            }
        }
        // Defensive copy
        return new ParameterList(parameters);
    }

    private static Parameter.Type inferType(String name) {
        if (name.contains("bias")) {
            return Parameter.Type.BIAS;
        } else if (name.contains("gamma")) {
            return Parameter.Type.GAMMA;
        } else if (name.contains("beta")) {
            return Parameter.Type.BETA;
        } else if (name.contains("moving_mean") || name.contains("running_mean")) {
            return Parameter.Type.RUNNING_MEAN;
        } else if (name.contains("moving_var") || name.contains("running_var")) {
            return Parameter.Type.RUNNING_VAR;
        } else if (name.contains("weight")) {
            return Parameter.Type.WEIGHT;
        }
        return Parameter.Type.OTHER;
    }

    /** {@inheritDoc} */
    @Override
    public PairList<String, Shape> describeOutput() {
        if (outputDescriptions == null) {
            logger.warn(
                    "Output shapes are unknown, please run predict or forward once"
                            + " and call describeOutput again.");
        }
        return outputDescriptions;
    }

    /** {@inheritDoc} */
    @Override
    public Shape[] getOutputShapes(Shape[] inputShapes) {
        try (NDManager manager = NDManager.newBaseManager()) {
            NDList list = new NDList();
            // TODO: Only tested for float32
            for (Shape shape : inputShapes) {
                list.add(manager.ones(shape));
            }
            NDList result = forwardInternal(new ParameterStore(manager, false), list, false, null);
            return result.stream().map(NDArray::getShape).toArray(Shape[]::new);
        }
    }

    /** {@inheritDoc} */
    @Override
    public Shape[] getOutputShapes(Shape[] inputShapes, DataType[] dataTypes) {
        try (NDManager manager = NDManager.newBaseManager()) {
            NDList list = new NDList();
            for (int i = 0; i < inputShapes.length; i++) {
                list.add(
                        manager.ones(
                                inputShapes[i],
                                dataTypes == null ? DataType.FLOAT32 : dataTypes[i]));
            }
            NDList result = forwardInternal(new ParameterStore(manager, false), list, false, null);
            return result.stream().map(NDArray::getShape).toArray(Shape[]::new);
        }
    }

    /** {@inheritDoc} */
    @Override
    public void saveParameters(DataOutputStream os) throws IOException {
        os.writeByte(version);
        JniUtils.writeModule(this, os, true);
    }

    /** {@inheritDoc} */
    @Override
    public void loadParameters(NDManager manager, DataInputStream is)
            throws IOException, MalformedModelException {
        byte loadVersion = is.readByte();
        if (loadVersion != version) {
            throw new MalformedModelException("Unsupported encoding version: " + loadVersion);
        }
        long rawHandle = JniUtils.loadModuleHandle(is, manager.getDevice(), true, true);
        this.handle = new AtomicReference<>(rawHandle);
        uid = String.valueOf(rawHandle);
        manager.attachInternal(uid, this);
    }

    /**
     * Get the native PyTorch model pointer.
     *
     * @return the pointer
     */
    public Long getHandle() {
        Long reference = handle.get();
        if (reference == null) {
            throw new IllegalStateException("PyTorch model handle has been released!");
        }
        return reference;
    }
}
