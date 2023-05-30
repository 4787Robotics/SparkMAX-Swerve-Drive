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
package ai.djl.tensorflow.integration.modality.cv;

import ai.djl.Application;
import ai.djl.ModelException;
import ai.djl.inference.Predictor;
import ai.djl.modality.Classifications;
import ai.djl.modality.cv.Image;
import ai.djl.modality.cv.ImageFactory;
import ai.djl.modality.cv.output.DetectedObjects;
import ai.djl.ndarray.NDArray;
import ai.djl.ndarray.NDList;
import ai.djl.ndarray.NDManager;
import ai.djl.ndarray.types.DataType;
import ai.djl.ndarray.types.Shape;
import ai.djl.repository.zoo.Criteria;
import ai.djl.repository.zoo.ZooModel;
import ai.djl.testing.TestRequirements;
import ai.djl.training.util.ProgressBar;
import ai.djl.translate.NoopTranslator;
import ai.djl.translate.TranslateException;
import ai.djl.util.Pair;

import org.testng.Assert;
import org.testng.annotations.BeforeClass;
import org.testng.annotations.Test;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.stream.Collectors;

public class TfSsdTest {

    @BeforeClass
    public void setUp() {
        TestRequirements.notArm();
    }

    @Test
    public void testTfSSD() throws IOException, ModelException, TranslateException {
        Criteria<Image, DetectedObjects> criteria =
                Criteria.builder()
                        .optApplication(Application.CV.OBJECT_DETECTION)
                        .setTypes(Image.class, DetectedObjects.class)
                        .optArtifactId("ssd")
                        .optFilter("backbone", "mobilenet_v2")
                        .optEngine("TensorFlow")
                        .optProgress(new ProgressBar())
                        .build();

        Path file = Paths.get("../../../examples/src/test/resources/dog_bike_car.jpg");
        Image img = ImageFactory.getInstance().fromFile(file);
        try (ZooModel<Image, DetectedObjects> model = criteria.loadModel();
                Predictor<Image, DetectedObjects> predictor = model.newPredictor()) {
            Assert.assertEquals(model.describeInput().get(0).getValue(), new Shape(-1, -1, -1, 3));
            for (Pair<String, Shape> pair : model.describeOutput()) {
                switch (pair.getKey()) {
                    case "box":
                    case "detection_boxes":
                        Assert.assertEquals(pair.getValue(), new Shape(-1, 4));
                        break;
                    case "label":
                    case "score":
                    case "detection_class_entities":
                    case "detection_class_labels":
                    case "detection_class_names":
                    case "detection_scores":
                        Assert.assertEquals(pair.getValue(), new Shape(-1, 1));
                        break;
                    default:
                        throw new IllegalStateException("Unexpected output name: " + pair.getKey());
                }
            }

            DetectedObjects result = predictor.predict(img);
            List<String> classes =
                    result.items().stream()
                            .map(Classifications.Classification::getClassName)
                            .collect(Collectors.toList());
            Assert.assertTrue(classes.contains("Dog"));
            Assert.assertTrue(classes.contains("Bicycle"));
            Assert.assertTrue(classes.contains("Car"));
            saveBoundingBoxImage(img, result);
        }
    }

    @Test
    public void testStringInputOutput() throws IOException, ModelException, TranslateException {
        Criteria<NDList, NDList> criteria =
                Criteria.builder()
                        .optApplication(Application.CV.OBJECT_DETECTION)
                        .setTypes(NDList.class, NDList.class)
                        .optArtifactId("ssd")
                        .optFilter("backbone", "mobilenet_v2")
                        .optEngine("TensorFlow")
                        .optProgress(new ProgressBar())
                        .optTranslator(new NoopTranslator())
                        .build();

        try (ZooModel<NDList, NDList> model = criteria.loadModel();
                Predictor<NDList, NDList> predictor = model.newPredictor();
                NDManager manager = NDManager.newBaseManager()) {
            NDArray array = manager.zeros(new Shape(1, 224, 224, 3));
            NDList output = predictor.predict(new NDList(array));
            Assert.assertEquals(output.size(), 5);
            NDArray entities = output.get("detection_class_entities");
            Assert.assertEquals(entities.getDataType(), DataType.STRING);
        }
    }

    private static void saveBoundingBoxImage(Image img, DetectedObjects detection)
            throws IOException {
        Path outputDir = Paths.get("build/output");
        Files.createDirectories(outputDir);

        img.drawBoundingBoxes(detection);

        Path imagePath = outputDir.resolve("detected-dog_bike_car.png");
        // OpenJDK can't save jpg with alpha channel
        img.save(Files.newOutputStream(imagePath), "png");
    }
}
