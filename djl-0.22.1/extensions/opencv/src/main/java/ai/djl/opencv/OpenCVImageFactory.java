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
package ai.djl.opencv;

import ai.djl.modality.cv.Image;
import ai.djl.modality.cv.ImageFactory;
import ai.djl.modality.cv.util.NDImageUtils;
import ai.djl.ndarray.NDArray;
import ai.djl.ndarray.types.DataType;
import ai.djl.ndarray.types.Shape;
import ai.djl.util.Utils;

import nu.pattern.OpenCV;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.nio.file.Path;

/** {@code OpenCVImageFactory} is a high performance implementation of {@link ImageFactory}. */
public class OpenCVImageFactory extends ImageFactory {

    static {
        OpenCV.loadLocally();
        if (System.getProperty("apple.awt.UIElement") == null) {
            // disables coffee cup image showing up on macOS
            System.setProperty("apple.awt.UIElement", "true");
        }
    }

    /** {@inheritDoc} */
    @Override
    public Image fromFile(Path path) throws IOException {
        // Load image without alpha channel
        Mat img = Imgcodecs.imread(path.toAbsolutePath().toString());
        if (img.empty()) {
            throw new IOException("Read image failed: " + path);
        }
        return new OpenCVImage(img);
    }

    /** {@inheritDoc} */
    @Override
    public Image fromInputStream(InputStream is) throws IOException {
        byte[] buf = Utils.toByteArray(is);
        Mat mat = new MatOfByte(buf);
        Mat img = Imgcodecs.imdecode(mat, Imgcodecs.IMREAD_COLOR);
        if (img.empty()) {
            throw new IOException("Read image failed.");
        }
        return new OpenCVImage(img);
    }

    /** {@inheritDoc} */
    @Override
    public Image fromImage(Object image) {
        return new OpenCVImage((Mat) image);
    }

    /** {@inheritDoc} */
    @Override
    public Image fromNDArray(NDArray array) {
        Shape shape = array.getShape();
        if (shape.dimension() == 4) {
            throw new UnsupportedOperationException("Batch is not supported");
        }
        array = array.toType(DataType.UINT8, false);
        boolean grayScale = shape.get(0) == 1 || shape.get(2) == 1;
        if (grayScale) {
            // expected CHW
            int width = Math.toIntExact(shape.get(2));
            int height = Math.toIntExact(shape.get(1));
            Mat img = new Mat(height, width, CvType.CV_8UC1);
            img.put(0, 0, array.toByteArray());
            return new OpenCVImage(img);
        }
        if (NDImageUtils.isCHW(shape)) {
            array = array.transpose(1, 2, 0);
            shape = array.getShape();
        }
        int width = Math.toIntExact(shape.get(1));
        int height = Math.toIntExact(shape.get(0));
        Mat img = new Mat(height, width, CvType.CV_8UC3);
        img.put(0, 0, array.toByteArray());
        Imgproc.cvtColor(img, img, Imgproc.COLOR_RGB2BGR);
        return new OpenCVImage(img);
    }

    /** {@inheritDoc} */
    @Override
    public Image fromPixels(int[] pixels, int width, int height) {
        Mat img = new Mat(height, width, CvType.CV_8UC4);
        byte[] data = new byte[width * height * 4];
        IntBuffer buf = ByteBuffer.wrap(data).asIntBuffer();
        for (int pixel : pixels) {
            int r = pixel >> 8 & 0xff00;
            int g = pixel << 8 & 0xff0000;
            int b = pixel << 24 & 0xff000000;
            buf.put(pixel >>> 24 | b | g | r);
        }
        img.put(0, 0, data);
        return new OpenCVImage(img);
    }
}
