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
package ai.djl.tensorflow.integration;

import ai.djl.engine.Engine;
import ai.djl.tensorflow.engine.TfEngine;
import ai.djl.testing.CoverageUtils;
import ai.djl.testing.TestRequirements;

import org.testng.annotations.Test;

import java.io.IOException;
import java.net.URISyntaxException;

public class TfCoverageTest {

    @Test
    public void test() throws IOException, ReflectiveOperationException, URISyntaxException {
        TestRequirements.notArm();

        // tensorflow-engine
        Engine.getEngine("TensorFlow");
        CoverageUtils.testGetterSetters(TfEngine.class);
    }
}
