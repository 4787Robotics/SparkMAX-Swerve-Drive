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
package ai.djl.basicdataset;

import ai.djl.basicdataset.nlp.AmazonReview;
import ai.djl.ndarray.NDManager;
import ai.djl.training.dataset.Record;
import ai.djl.translate.TranslateException;

import org.testng.Assert;
import org.testng.annotations.Test;

import java.io.IOException;

public class AmazonReviewsTest {

    @Test
    public void testAmazonReviews() throws IOException, TranslateException {
        try (NDManager manager = NDManager.newBaseManager()) {
            AmazonReview dataset =
                    AmazonReview.builder()
                            .setSampling(1, false)
                            .addCategoricalFeature("marketplace")
                            .addNumericLabel("star_rating")
                            .optLimit(2)
                            .build();
            dataset.prepare();

            Record record = dataset.get(manager, 0);
            Assert.assertEquals(
                    record.getData().singletonOrThrow().toFloatArray(), new float[] {1});
            Assert.assertEquals(record.getLabels().singletonOrThrow().getFloat(), 4.0);
        }
    }
}
