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
import ai.djl.basicdataset.tabular.DailyDelhiClimate;
import ai.djl.ndarray.NDList;
import ai.djl.ndarray.NDManager;
import ai.djl.nn.Blocks;
import ai.djl.nn.Parameter;
import ai.djl.training.DefaultTrainingConfig;
import ai.djl.training.Trainer;
import ai.djl.training.TrainingConfig;
import ai.djl.training.dataset.Batch;
import ai.djl.training.dataset.Dataset;
import ai.djl.training.dataset.Record;
import ai.djl.training.initializer.Initializer;
import ai.djl.training.loss.Loss;
import ai.djl.translate.TranslateException;

import org.testng.Assert;
import org.testng.annotations.Test;

import java.io.IOException;

public class DailyDelhiClimateTest {

    @Test
    public void testDailyDelhiClimateRemote() throws IOException, TranslateException {
        TrainingConfig config =
                new DefaultTrainingConfig(Loss.softmaxCrossEntropyLoss())
                        .optInitializer(Initializer.ONES, Parameter.Type.WEIGHT);

        try (Model model = Model.newInstance("model")) {
            model.setBlock(Blocks.identityBlock());

            NDManager manager = model.getNDManager();
            // path of directory
            DailyDelhiClimate dailyDelhiClimate =
                    DailyDelhiClimate.builder()
                            .optUsage(Dataset.Usage.TEST)
                            .addFeature("humidity")
                            .addFeature("wind_speed")
                            .noLabels()
                            .setSampling(32, true)
                            .build();

            dailyDelhiClimate.prepare();

            long size = dailyDelhiClimate.size();
            Assert.assertEquals(size, 114);

            Record record = dailyDelhiClimate.get(manager, 0);
            NDList data = record.getData();
            NDList labels = record.getLabels();
            Assert.assertEquals(data.head().toFloatArray(), new float[] {85.86957f, 2.7434783f});
            Assert.assertEquals(labels.size(), 0);

            try (Trainer trainer = model.newTrainer(config)) {
                Batch batch = trainer.iterateDataset(dailyDelhiClimate).iterator().next();
                Assert.assertEquals(batch.getData().size(), 1);
                Assert.assertEquals(batch.getLabels().size(), 0);
                batch.close();
            }
        }
    }
}
