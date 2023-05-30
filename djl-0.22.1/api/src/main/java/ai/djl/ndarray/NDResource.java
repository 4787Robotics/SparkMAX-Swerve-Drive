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
package ai.djl.ndarray;

import java.util.List;

/** An object which is managed by an {@link NDManager} and tracks the manager it is attached to. */
public interface NDResource extends AutoCloseable {

    /**
     * Returns the {@link NDManager} that manages this.
     *
     * @return the {@link NDManager} that manages this.
     */
    NDManager getManager();

    /**
     * Returns the {@link NDArray} or {@link NDArray}s contained within this resource.
     *
     * @return the {@link NDArray} or {@link NDArray}s contained within this resource
     */
    List<NDArray> getResourceNDArrays();

    /**
     * Attaches this {@link NDResource} to the specified {@link NDManager}.
     *
     * <p>Attached resource will be closed when the {@link NDManager} is closed.
     *
     * @param manager the {@link NDManager} to be attached to
     */
    void attach(NDManager manager);

    /**
     * Attaches this {@link NDResource} to the specified {@link NDManager} from which it was
     * previously detached and temp-attached to the current manager of this resource. This is
     * functionally equivalent to the attach method, however the cap-state disregarded when adding
     * the resource back to the original manager.
     *
     * @param manager the {@link NDManager} to be attached to
     */
    default void returnResource(NDManager manager) {
        attach(manager);
    }

    /**
     * Temporarily attaches this {@link NDResource} to the specified {@link NDManager}.
     *
     * <p>Attached resource will be returned to the original manager when the {@link NDManager} is
     * closed.
     *
     * @param manager the {@link NDManager} to be attached to
     */
    void tempAttach(NDManager manager);

    /**
     * Detaches the {@link NDResource} from current {@link NDManager}'s lifecycle.
     *
     * <p>This becomes un-managed and it is the user's responsibility to close this. Failure to
     * close the resource might cause your machine to run out of native memory.
     *
     * @see NDManager
     */
    void detach();

    /** {@inheritDoc} */
    @Override
    void close();
}
