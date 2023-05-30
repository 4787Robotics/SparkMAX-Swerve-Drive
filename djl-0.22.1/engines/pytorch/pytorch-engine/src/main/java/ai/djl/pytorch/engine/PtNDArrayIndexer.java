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

import ai.djl.ndarray.NDArray;
import ai.djl.ndarray.index.NDArrayIndexer;
import ai.djl.ndarray.index.NDIndex;
import ai.djl.ndarray.index.dim.NDIndexBooleans;
import ai.djl.ndarray.index.full.NDIndexFullPick;
import ai.djl.ndarray.index.full.NDIndexFullSlice;
import ai.djl.ndarray.index.full.NDIndexFullTake;
import ai.djl.ndarray.types.Shape;
import ai.djl.pytorch.jni.JniUtils;

import java.util.Stack;

/** The {@link NDArrayIndexer} used by the {@link PtNDArray}. */
public class PtNDArrayIndexer extends NDArrayIndexer {

    private PtNDManager manager;

    PtNDArrayIndexer(PtNDManager manager) {
        this.manager = manager;
    }

    /** {@inheritDoc} */
    @Override
    public NDArray get(NDArray array, NDIndexFullPick fullPick) {
        return JniUtils.pick(
                manager.from(array), manager.from(fullPick.getIndices()), fullPick.getAxis());
    }

    /** {@inheritDoc} */
    @Override
    public NDArray get(NDArray array, NDIndexFullTake fullTake) {
        return JniUtils.take(manager.from(array), manager.from(fullTake.getIndices()), manager);
    }

    /** {@inheritDoc} */
    @Override
    public NDArray get(NDArray array, NDIndexFullSlice fullSlice) {
        long[] min = fullSlice.getMin();
        long[] max = fullSlice.getMax();
        long[] step = fullSlice.getStep();
        try (PtNDArray res = JniUtils.index(manager.from(array), min, max, step, manager)) {
            return res.squeeze(fullSlice.getToSqueeze());
        }
    }

    /** {@inheritDoc} */
    @Override
    public NDArray get(NDArray array, NDIndex index) {
        if (index.getRank() == 0) {
            if (array.getShape().isScalar()) {
                return array.getManager() == manager
                        ? array.duplicate()
                        : manager.create(
                                array.toByteBuffer(), array.getShape(), array.getDataType());
            }
            index.addAllDim();
        }

        if (array == null || array instanceof PtNDArray) {
            return JniUtils.indexAdv((PtNDArray) array, index, manager);
        } else {
            PtNDArray arrayNew =
                    manager.create(array.toByteBuffer(), array.getShape(), array.getDataType());
            return JniUtils.indexAdv(arrayNew, index, manager);
        }
    }

    /** {@inheritDoc} */
    @Override
    public void set(NDArray array, NDIndex index, Object data) {
        PtNDArray ptArray =
                array instanceof PtNDArray
                        ? (PtNDArray) array
                        : manager.create(
                                array.toByteBuffer(), array.getShape(), array.getDataType());

        if (data instanceof Number) {
            JniUtils.indexAdvPut(ptArray, index, (PtNDArray) manager.create((Number) data));
        } else if (data instanceof NDArray) {
            JniUtils.indexAdvPut(ptArray, index, manager.from((NDArray) data));
        } else {
            throw new IllegalArgumentException(
                    "The type of value to assign cannot be other than NDArray and Number.");
        }
    }

    /** {@inheritDoc} */
    @Override
    public void set(NDArray array, NDIndexFullSlice fullSlice, NDArray value) {
        Stack<NDArray> prepareValue = new Stack<>();
        prepareValue.add(value);
        prepareValue.add(prepareValue.peek().toDevice(array.getDevice(), false));
        // Deal with the case target: (1, 10, 1), original (10)
        // try to find (10, 1) and reshape (10) to that
        Shape targetShape = fullSlice.getShape();
        while (targetShape.size() > value.size()) {
            targetShape = targetShape.slice(1);
        }
        prepareValue.add(prepareValue.peek().reshape(targetShape));
        prepareValue.add(prepareValue.peek().broadcast(fullSlice.getShape()));
        JniUtils.indexSet(
                manager.from(array),
                manager.from(prepareValue.peek()),
                fullSlice.getMin(),
                fullSlice.getMax(),
                fullSlice.getStep());
        for (NDArray toClean : prepareValue) {
            if (toClean != value) {
                toClean.close();
            }
        }
    }

    /** {@inheritDoc} */
    @Override
    public void set(NDArray array, NDIndexBooleans indices, NDArray value) {
        try (NDArray mask = indices.getIndex()) {
            JniUtils.booleanMaskSet(manager.from(array), manager.from(value), manager.from(mask));
        }
    }

    /** {@inheritDoc} */
    @Override
    public void set(NDArray array, NDIndexFullSlice fullSlice, Number value) {
        set(array, fullSlice, array.getManager().create(value));
    }
}
