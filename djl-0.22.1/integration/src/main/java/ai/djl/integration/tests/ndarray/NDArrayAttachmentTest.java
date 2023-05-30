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
package ai.djl.integration.tests.ndarray;

import ai.djl.integration.util.TestUtils;
import ai.djl.ndarray.NDArray;
import ai.djl.ndarray.NDManager;
import ai.djl.ndarray.index.NDIndex;
import ai.djl.ndarray.types.Shape;

import org.testng.Assert;
import org.testng.annotations.Test;

public class NDArrayAttachmentTest {

    @Test
    public void testReturnResource() {
        try (NDManager manager = NDManager.newBaseManager(TestUtils.getEngine())) {
            NDArray array3x4 = manager.ones(new Shape(3, 4));
            try (NDManager subManager = NDManager.newBaseManager(TestUtils.getEngine())) {
                array3x4.tempAttach(subManager);
                Assert.assertEquals(array3x4.getManager(), subManager);
            }
            Assert.assertEquals(array3x4.getManager(), manager);
        }
    }

    @Test
    public void testIndexationUsesSpecificManager() {
        // Hybrid engine's operation will change its' NDManager
        TestUtils.requiresEngine("MXNet", "PyTorch", "TensorFlow");

        try (NDManager manager = NDManager.newBaseManager(TestUtils.getEngine())) {
            NDArray array3x4 = manager.ones(new Shape(3, 4));
            array3x4.setName("Test()");
            NDArray array4 = array3x4.get(1);
            Assert.assertEquals(array4.getManager(), manager);
            try (NDManager subManager = NDManager.newBaseManager(TestUtils.getEngine())) {
                NDArray array4sub1 = array3x4.get(subManager, 1);
                Assert.assertEquals(array4sub1.getManager(), subManager);
                NDArray array4sub2 = array3x4.get(subManager, new NDIndex(1));
                Assert.assertEquals(array4sub2.getManager(), subManager);
            }
        }
    }

    @Test(expectedExceptions = IllegalStateException.class)
    public void testCannotUseCappedManager() {
        try (NDManager manager = NDManager.newBaseManager(TestUtils.getEngine())) {
            manager.cap();
            manager.ones(new Shape(3, 4));
        }
    }

    @Test(expectedExceptions = IllegalStateException.class)
    public void testIndexationCannotUseCappedManager() {
        try (NDManager manager = NDManager.newBaseManager(TestUtils.getEngine())) {
            NDArray array3x4 = manager.ones(new Shape(3, 4));
            array3x4.setName("Test()");
            manager.cap();
            array3x4.get(1);
        }
    }

    @Test
    public void testIndexationUsesSpecificUncappedManager() {
        // Hybrid engine's operation will change its' NDManager
        TestUtils.requiresEngine("MXNet", "PyTorch", "TensorFlow");

        try (NDManager manager = NDManager.newBaseManager(TestUtils.getEngine())) {
            NDArray array3x4 = manager.ones(new Shape(3, 4));
            array3x4.setName("Test()");
            manager.cap();
            try (NDManager subManager = NDManager.newBaseManager(TestUtils.getEngine())) {
                NDArray array4sub1 = array3x4.get(subManager, 1);
                Assert.assertEquals(array4sub1.getManager(), subManager);
                array3x4.tempAttach(subManager);
                Assert.assertEquals(array3x4.getManager(), subManager);
            }
        }
    }
}
