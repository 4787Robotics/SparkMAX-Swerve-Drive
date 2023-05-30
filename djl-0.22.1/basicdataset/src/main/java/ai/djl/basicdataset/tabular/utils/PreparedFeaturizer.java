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
package ai.djl.basicdataset.tabular.utils;

import java.util.List;

/** A {@link Featurizer} that must be prepared with the possible feature values before use. */
public interface PreparedFeaturizer extends Featurizer {

    /**
     * Prepares the featurizer with the list of possible inputs.
     *
     * @param inputs the possible inputs
     */
    void prepare(List<String> inputs);
}
