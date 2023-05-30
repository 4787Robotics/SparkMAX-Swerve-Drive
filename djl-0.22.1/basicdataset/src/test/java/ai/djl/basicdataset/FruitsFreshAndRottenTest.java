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
package ai.djl.basicdataset;

import ai.djl.Model;
import ai.djl.basicdataset.cv.classification.FruitsFreshAndRotten;
import ai.djl.modality.cv.transform.Resize;
import ai.djl.nn.Blocks;
import ai.djl.training.DefaultTrainingConfig;
import ai.djl.training.Trainer;
import ai.djl.training.TrainingConfig;
import ai.djl.training.dataset.Batch;
import ai.djl.training.dataset.Dataset;
import ai.djl.training.loss.Loss;
import ai.djl.translate.TranslateException;

import org.testng.Assert;
import org.testng.annotations.Test;

import java.io.IOException;

public class FruitsFreshAndRottenTest {

    @Test
    public void testFruitsFreshAndRottenRemote() throws IOException, TranslateException {
        TrainingConfig config = new DefaultTrainingConfig(Loss.softmaxCrossEntropyLoss());

        try (Model model = Model.newInstance("model")) {
            model.setBlock(Blocks.identityBlock());

            FruitsFreshAndRotten dataset =
                    FruitsFreshAndRotten.builder()
                            .optUsage(Dataset.Usage.TEST)
                            .optArtifactId("fruit-unittest")
                            .addTransform(new Resize(256, 256))
                            .setSampling(32, true)
                            .build();
            dataset.prepare();

            try (Trainer trainer = model.newTrainer(config)) {
                Batch batch = trainer.iterateDataset(dataset).iterator().next();
                Assert.assertEquals(batch.getData().size(), 1);
                Assert.assertEquals(batch.getLabels().size(), 1);
                batch.close();
            }
        }
    }
}
