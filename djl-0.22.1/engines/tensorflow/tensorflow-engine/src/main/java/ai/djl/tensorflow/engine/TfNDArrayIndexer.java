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
package ai.djl.tensorflow.engine;

import ai.djl.ndarray.NDArray;
import ai.djl.ndarray.index.NDArrayIndexer;
import ai.djl.ndarray.index.full.NDIndexFullPick;
import ai.djl.ndarray.index.full.NDIndexFullSlice;
import ai.djl.ndarray.index.full.NDIndexFullTake;

/** The {@link NDArrayIndexer} used by the {@link TfNDArray}. */
public class TfNDArrayIndexer extends NDArrayIndexer {

    private TfNDManager manager;

    TfNDArrayIndexer(TfNDManager manager) {
        this.manager = manager;
    }

    /** {@inheritDoc} */
    @Override
    public NDArray get(NDArray array, NDIndexFullPick fullPick) {
        throw new UnsupportedOperationException("Not implemented");
    }

    /** {@inheritDoc} */
    @Override
    public NDArray get(NDArray array, NDIndexFullTake fullTake) {
        throw new UnsupportedOperationException("Not implemented");
    }

    /** {@inheritDoc} */
    @Override
    public NDArray get(NDArray array, NDIndexFullSlice fullSlice) {
        array = manager.from(array);
        int[] toSqueeze = fullSlice.getToSqueeze();
        try (NDArray begin = manager.create(fullSlice.getMin());
                NDArray end = manager.create(fullSlice.getMax());
                NDArray step = manager.create(fullSlice.getStep())) {
            NDArray result =
                    manager.opExecutor("StridedSlice")
                            .addInput(array)
                            .addInput(begin)
                            .addInput(end)
                            .addInput(step)
                            .buildSingletonOrThrow();
            if (toSqueeze.length > 0) {
                NDArray oldResult = result;
                result = result.squeeze(toSqueeze);
                oldResult.close();
            }
            return result;
        }
    }

    /** {@inheritDoc} */
    @Override
    public void set(NDArray array, NDIndexFullSlice fullSlice, NDArray value) {
        throw new UnsupportedOperationException("Tensor cannot be modified after creation");
    }

    /** {@inheritDoc} */
    @Override
    public void set(NDArray array, NDIndexFullSlice fullSlice, Number value) {
        throw new UnsupportedOperationException("Tensor cannot be modified after creation");
    }
}
