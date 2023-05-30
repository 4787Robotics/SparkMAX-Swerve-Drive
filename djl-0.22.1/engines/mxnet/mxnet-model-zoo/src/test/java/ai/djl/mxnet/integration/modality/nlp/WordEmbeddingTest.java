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
package ai.djl.mxnet.integration.modality.nlp;

import ai.djl.ModelException;
import ai.djl.modality.nlp.embedding.EmbeddingException;
import ai.djl.modality.nlp.embedding.ModelZooTextEmbedding;
import ai.djl.ndarray.NDList;
import ai.djl.ndarray.NDManager;
import ai.djl.repository.zoo.Criteria;
import ai.djl.repository.zoo.ZooModel;
import ai.djl.testing.TestRequirements;

import org.testng.Assert;
import org.testng.annotations.Test;

import java.io.IOException;
import java.util.Collections;

public class WordEmbeddingTest {

    @Test
    public void testGlove() throws IOException, ModelException, EmbeddingException {
        TestRequirements.notArm();

        Criteria<String, NDList> criteria =
                Criteria.builder()
                        .setTypes(String.class, NDList.class)
                        .optArtifactId("ai.djl.mxnet:glove")
                        .build();
        try (ZooModel<String, NDList> model = criteria.loadModel()) {
            try (ModelZooTextEmbedding wordEmbedding = new ModelZooTextEmbedding(model)) {
                NDManager manager = model.getNDManager();
                NDList result =
                        new NDList(
                                wordEmbedding.embedText(manager, Collections.singletonList("the")));
                Assert.assertEquals(result.singletonOrThrow().sum().getFloat(), 0.0, .01);
            }
        }
    }
}
