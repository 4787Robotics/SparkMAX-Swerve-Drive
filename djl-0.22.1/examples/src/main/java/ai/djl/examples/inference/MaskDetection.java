/*
 * Copyright 2023 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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
package ai.djl.examples.inference;

import ai.djl.ModelException;
import ai.djl.inference.Predictor;
import ai.djl.modality.cv.Image;
import ai.djl.modality.cv.ImageFactory;
import ai.djl.modality.cv.output.DetectedObjects;
import ai.djl.modality.cv.translator.YoloV5TranslatorFactory;
import ai.djl.repository.zoo.Criteria;
import ai.djl.repository.zoo.ZooModel;
import ai.djl.training.util.ProgressBar;
import ai.djl.translate.TranslateException;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * An example of inference using an object detection model.
 *
 * <p>See this <a
 * href="https://github.com/deepjavalibrary/djl/blob/master/examples/docs/mask_detection.md">doc</a>
 * for information about this example.
 */
public final class MaskDetection {

    private static final Logger logger = LoggerFactory.getLogger(MaskDetection.class);

    private MaskDetection() {}

    public static void main(String[] args) throws IOException, ModelException, TranslateException {
        DetectedObjects detection = MaskDetection.predict();
        logger.info("{}", detection);
    }

    public static DetectedObjects predict() throws IOException, ModelException, TranslateException {
        // To feed in local image, use ImageFactory.getInstance().fromFile(...)
        String imageUrl = "https://resources.djl.ai/images/face_mask_detection/face_mask.png";
        Image img = ImageFactory.getInstance().fromUrl(imageUrl);

        // modelUrl can be replaced to local onnx model file
        String modelUrl = "https://resources.djl.ai/demo/onnxruntime/face_mask_detection.zip";
        Criteria<Image, DetectedObjects> criteria =
                Criteria.builder()
                        .setTypes(Image.class, DetectedObjects.class)
                        .optModelUrls(modelUrl)
                        .optEngine("OnnxRuntime")
                        .optTranslatorFactory(new YoloV5TranslatorFactory())
                        .optProgress(new ProgressBar())
                        .optArgument("optApplyRatio", true) // post process
                        .optArgument("rescale", true) // post process
                        .build();

        try (ZooModel<Image, DetectedObjects> model = criteria.loadModel()) {
            try (Predictor<Image, DetectedObjects> predictor = model.newPredictor()) {
                DetectedObjects detection = predictor.predict(img);
                String outputDir = "build/output";
                saveBoundingBoxImage(img, detection, outputDir);
                return detection;
            }
        }
    }

    private static void saveBoundingBoxImage(Image img, DetectedObjects detection, String outputDir)
            throws IOException {
        Path outputPath = Paths.get(outputDir);
        Files.createDirectories(outputPath);

        img.drawBoundingBoxes(detection);

        Path imagePath = outputPath.resolve("face_mask_result.png");
        // OpenJDK can't save jpg with alpha channel
        img.save(Files.newOutputStream(imagePath), "png");
        logger.info("Detected objects image has been saved in: {}", imagePath);
    }
}
