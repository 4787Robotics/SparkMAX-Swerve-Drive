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
package ai.djl.util.passthrough;

import ai.djl.Device;
import ai.djl.engine.Engine;
import ai.djl.ndarray.BaseNDManager;
import ai.djl.ndarray.NDArray;
import ai.djl.ndarray.NDList;
import ai.djl.ndarray.NDManager;
import ai.djl.ndarray.NDResource;
import ai.djl.ndarray.types.DataType;
import ai.djl.ndarray.types.Shape;
import ai.djl.util.PairList;

import java.nio.Buffer;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.Charset;
import java.nio.file.Path;
import java.util.Collections;
import java.util.List;

/** An {@link NDManager} that does nothing, for use in extensions and hybrid engines. */
public final class PassthroughNDManager implements NDManager {

    private static final String UNSUPPORTED = "Not supported by PassthroughNDManager";
    public static final PassthroughNDManager INSTANCE = new PassthroughNDManager();

    private Engine engine;
    private Device device;

    /**
     * Constructs a new {@code PassthroughNDManager} instance.
     *
     * @param engine the {@link Engine} associated with this manager
     * @param device the default {@link Device}
     */
    public PassthroughNDManager(Engine engine, Device device) {
        this.engine = engine;
        this.device = device == null ? engine.defaultDevice() : device;
    }

    private PassthroughNDManager() {
        device = Device.cpu();
    }

    /** {@inheritDoc} */
    @Override
    public Device defaultDevice() {
        if (engine != null) {
            return engine.defaultDevice();
        }
        return device;
    }

    /** {@inheritDoc} */
    @Override
    public ByteBuffer allocateDirect(int capacity) {
        return ByteBuffer.allocateDirect(capacity).order(ByteOrder.nativeOrder());
    }

    /** {@inheritDoc} */
    @Override
    public NDArray from(NDArray array) {
        if (array == null || array instanceof PassthroughNDArray) {
            return array;
        }
        return create(array.toByteBuffer(), array.getShape(), array.getDataType());
    }

    /**
     * Creates a new {@link PassthroughNDArray}.
     *
     * @param object the object to store
     * @return a new {@code PassthroughNDArray}
     */
    public PassthroughNDArray create(Object object) {
        return new PassthroughNDArray(this, object);
    }

    /** {@inheritDoc} */
    @Override
    public NDArray create(Buffer data, Shape shape, DataType dataType) {
        int size = Math.toIntExact(shape.size());
        BaseNDManager.validateBuffer(data, dataType, size);
        if (data instanceof ByteBuffer) {
            return new PassthroughNDArray(this, data, shape, dataType);
        }
        ByteBuffer bb = ByteBuffer.allocate(size * dataType.getNumOfBytes());
        bb.order(ByteOrder.nativeOrder());
        BaseNDManager.copyBuffer(data, bb);
        return new PassthroughNDArray(this, bb, shape, dataType);
    }

    /** {@inheritDoc} */
    @Override
    public NDArray create(String[] data, Charset charset, Shape shape) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public NDArray create(Shape shape, DataType dataType) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public NDArray createCSR(Buffer data, long[] indptr, long[] indices, Shape shape) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public NDArray createRowSparse(Buffer data, Shape dataShape, long[] indices, Shape shape) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public NDArray createCoo(Buffer data, long[][] indices, Shape shape) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public NDList load(Path path) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public void setName(String name) {}

    /** {@inheritDoc} */
    @Override
    public String getName() {
        return "PassthroughNDManager";
    }

    /** {@inheritDoc} */
    @Override
    public NDArray full(Shape shape, float value, DataType dataType) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public NDArray arange(float start, float stop, float step, DataType dataType) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public NDArray eye(int rows, int cols, int k, DataType dataType) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public NDArray linspace(float start, float stop, int num, boolean endpoint) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public NDArray randomInteger(long low, long high, Shape shape, DataType dataType) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public NDArray randomPermutation(long n) {
        throw new UnsupportedOperationException("Not supported!");
    }

    /** {@inheritDoc} */
    @Override
    public NDArray randomUniform(float low, float high, Shape shape, DataType dataType) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public NDArray randomNormal(float loc, float scale, Shape shape, DataType dataType) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public NDArray truncatedNormal(float loc, float scale, Shape shape, DataType dataType) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public NDArray randomMultinomial(int n, NDArray pValues) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public NDArray randomMultinomial(int n, NDArray pValues, Shape shape) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public NDArray sampleNormal(NDArray mu, NDArray sigma) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public NDArray sampleNormal(NDArray mu, NDArray sigma, Shape shape) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public NDArray samplePoisson(NDArray lam) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public NDArray samplePoisson(NDArray lam, Shape shape) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public NDArray sampleGamma(NDArray alpha, NDArray beta) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public NDArray sampleGamma(NDArray alpha, NDArray beta, Shape shape) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public boolean isOpen() {
        return true;
    }

    /** {@inheritDoc} */
    @Override
    public void cap() {}

    /** {@inheritDoc} */
    @Override
    public NDManager getParentManager() {
        return this;
    }

    /** {@inheritDoc} */
    @Override
    public NDManager newSubManager() {
        return this;
    }

    /** {@inheritDoc} */
    @Override
    public NDManager newSubManager(Device device) {
        return new PassthroughNDManager(engine, device);
    }

    /** {@inheritDoc} */
    @Override
    public Device getDevice() {
        return device;
    }

    /** {@inheritDoc} */
    @Override
    public List<NDArray> getManagedArrays() {
        return Collections.emptyList();
    }

    /** {@inheritDoc} */
    @Override
    public void attachInternal(String resourceId, AutoCloseable resource) {}

    /** {@inheritDoc} */
    @Override
    public void attachUncappedInternal(String resourceId, AutoCloseable resource) {}

    /** {@inheritDoc} */
    @Override
    public void tempAttachInternal(
            NDManager originalManager, String resourceId, NDResource resource) {}

    /** {@inheritDoc} */
    @Override
    public void detachInternal(String resourceId) {}

    /** {@inheritDoc} */
    @Override
    public void invoke(
            String operation, NDArray[] src, NDArray[] dest, PairList<String, ?> params) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public NDList invoke(String operation, NDList src, PairList<String, ?> params) {
        throw new UnsupportedOperationException(UNSUPPORTED);
    }

    /** {@inheritDoc} */
    @Override
    public Engine getEngine() {
        return engine;
    }

    /** {@inheritDoc} */
    @Override
    public void close() {}
}
