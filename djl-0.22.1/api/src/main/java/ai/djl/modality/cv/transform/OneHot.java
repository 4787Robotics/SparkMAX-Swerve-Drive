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
package ai.djl.modality.cv.transform;

import ai.djl.ndarray.NDArray;
import ai.djl.translate.Transform;

/** A {@link Transform} that converts the labels {@link NDArray} to one-hot labels. */
public class OneHot implements Transform {

    private int numClass;

    /**
     * Creates a {@code OneHot} {@link Transform} that converts the sparse label to one-hot label.
     *
     * @param numClass number of classes
     */
    public OneHot(int numClass) {
        this.numClass = numClass;
    }

    /** {@inheritDoc} */
    @Override
    public NDArray transform(NDArray array) {
        return array.oneHot(numClass);
    }
}
