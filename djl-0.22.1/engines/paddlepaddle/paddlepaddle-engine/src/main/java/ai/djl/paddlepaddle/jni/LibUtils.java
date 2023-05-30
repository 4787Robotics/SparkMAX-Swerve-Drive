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
package ai.djl.paddlepaddle.jni;

import ai.djl.util.ClassLoaderUtils;
import ai.djl.util.Platform;
import ai.djl.util.Utils;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.zip.GZIPInputStream;

/**
 * Utilities for finding the Paddle Engine binary on the System.
 *
 * <p>The Engine will be searched for in a variety of locations in the following order:
 *
 * <ol>
 *   <li>In the path specified by the Paddle_LIBRARY_PATH environment variable
 *   <li>In a jar file location in the classpath. These jars can be created with the paddle-native
 *       module.
 * </ol>
 */
@SuppressWarnings("MissingJavadocMethod")
public final class LibUtils {

    private static final Logger logger = LoggerFactory.getLogger(LibUtils.class);

    private static final String NATIVE_LIB_NAME = "paddle_inference";
    private static final String LIB_NAME = "djl_paddle";
    private static final Pattern VERSION_PATTERN =
            Pattern.compile("(\\d+\\.\\d+\\.\\d+)(-SNAPSHOT)?(-\\d+)?");

    private LibUtils() {}

    public static void loadLibrary() {
        String libName = LibUtils.findOverrideLibrary();
        AtomicBoolean fallback = new AtomicBoolean(false);
        if (libName == null) {
            libName = LibUtils.findLibraryInClasspath(fallback);
            if (libName == null) {
                throw new IllegalStateException("Native library not found");
            }
        }
        if (System.getProperty("os.name").startsWith("Linux")) {
            loadLinuxDependencies(libName);
        } else if (System.getProperty("os.name").startsWith("Win")) {
            loadWindowsDependencies(libName);
        } else if (System.getProperty("os.name").startsWith("Mac")) {
            loadMacOsDependencies(libName);
        }
        logger.debug("Now loading " + libName);
        System.load(libName); // NOPMD
        // TODO: change this part to load from cache directory
        Path nativeLibDir = Paths.get(libName).getParent();
        if (nativeLibDir == null || !nativeLibDir.toFile().isDirectory()) {
            throw new IllegalStateException("Native folder cannot be found");
        }
        libName = copyJniLibraryFromClasspath(nativeLibDir);
        logger.debug("Loading paddle library from: {}", libName);
        System.load(libName); // NOPMD
    }

    public static void loadLinuxDependencies(String libName) {
        Path libDir = Paths.get(libName).getParent();
        if (libDir != null) {
            logger.info(
                    "Paddle MKL/GPU requires user to set LD_LIBRARY_PATH="
                            + libDir
                            + ", the current one is set to: "
                            + Utils.getenv("LD_LIBRARY_PATH"));
            List<String> names =
                    Arrays.asList(
                            "libdnnl.so.2",
                            "libiomp5.so",
                            "libmklml_intel.so",
                            "libonnxruntime.so",
                            "libpaddle2onnx.so");
            names.forEach(
                    name -> {
                        Path path = libDir.resolve(name);
                        if (Files.isRegularFile(path)) {
                            String lib = path.toAbsolutePath().toString();
                            logger.debug("Now loading " + lib);
                            System.load(lib);
                        } else {
                            logger.debug(name + " is not found, skip loading...");
                        }
                    });
        }
    }

    public static void loadWindowsDependencies(String libName) {
        Path libDir = Paths.get(libName).getParent();
        List<String> names =
                Arrays.asList("openblas.dll", "mkldnn.dll", "onnxruntime.dll", "paddle2onnx.dll");
        names.forEach(
                name -> {
                    Path path = libDir.resolve(name);
                    if (Files.isRegularFile(path)) {
                        String lib = path.toAbsolutePath().toString();
                        logger.debug("Now loading " + lib);
                        System.load(lib);
                    } else {
                        logger.debug(name + " is not found, skip loading...");
                    }
                });
    }

    public static void loadMacOsDependencies(String libName) {
        Path libDir = Paths.get(libName).getParent();
        List<String> names = Arrays.asList("libonnxruntime.dylib", "libpaddle2onnx.dylib");
        names.forEach(
                name -> {
                    Path path = libDir.resolve(name);
                    if (Files.isRegularFile(path)) {
                        String lib = path.toAbsolutePath().toString();
                        logger.debug("Now loading " + lib);
                        System.load(lib);
                    } else {
                        logger.debug(name + " is not found, skip loading...");
                    }
                });
    }

    private static String findOverrideLibrary() {
        String libPath = Utils.getEnvOrSystemProperty("PADDLE_LIBRARY_PATH");
        if (libPath != null) {
            String libName = findLibraryInPath(libPath);
            if (libName != null) {
                return libName;
            }
        }

        libPath = System.getProperty("java.library.path");
        if (libPath != null) {
            return findLibraryInPath(libPath);
        }
        return null;
    }

    private static String copyJniLibraryFromClasspath(Path nativeDir) {
        String name = System.mapLibraryName(LIB_NAME);
        Platform platform = Platform.detectPlatform("paddlepaddle");
        String classifier = platform.getClassifier();
        String djlVersion = platform.getApiVersion();
        Path path = nativeDir.resolve(djlVersion + '-' + name);
        if (Files.exists(path)) {
            return path.toAbsolutePath().toString();
        }

        Path tmp = null;
        // Paddle GPU and CPU share the same jni so file
        String libPath = "jnilib/" + classifier + "/cpu/" + name;
        try (InputStream is = ClassLoaderUtils.getResourceAsStream(libPath)) {
            logger.info("Extracting {} to cache ...", libPath);
            tmp = Files.createTempFile(nativeDir, "jni", "tmp");
            Files.copy(is, tmp, StandardCopyOption.REPLACE_EXISTING);
            Utils.moveQuietly(tmp, path);
            return path.toAbsolutePath().toString();
        } catch (IOException e) {
            throw new IllegalStateException("Cannot copy jni files", e);
        } finally {
            if (tmp != null) {
                Utils.deleteQuietly(tmp);
            }
        }
    }

    private static synchronized String findLibraryInClasspath(AtomicBoolean fallback) {
        Platform platform = Platform.detectPlatform("paddlepaddle");
        if (platform.isPlaceholder()) {
            return downloadLibrary(platform, fallback);
        }
        String flavor = platform.getFlavor();
        if ("cpu".equals(flavor)) {
            fallback.set(true);
        }
        return loadLibraryFromClasspath(platform);
    }

    private static String loadLibraryFromClasspath(Platform platform) {
        Path tmp = null;
        try {
            String libName = System.mapLibraryName(NATIVE_LIB_NAME);
            Path cacheFolder = Utils.getEngineCacheDir("paddle");
            String version = platform.getVersion();
            String flavor = platform.getFlavor();
            String classifier = platform.getClassifier();
            Path dir = cacheFolder.resolve(version + '-' + flavor + '-' + classifier);
            logger.debug("Using cache dir: {}", dir);
            Path path = dir.resolve(libName);
            if (Files.exists(path)) {
                return path.toAbsolutePath().toString();
            }
            Files.createDirectories(cacheFolder);
            tmp = Files.createTempDirectory(cacheFolder, "tmp");
            for (String file : platform.getLibraries()) {
                String libPath = "native/lib/" + file;
                logger.info("Extracting {} to cache ...", file);
                if (file.endsWith(".gz")) {
                    // FIXME: temporary workaround for paddlepaddle-native-cu102:2.0.2
                    String f = file.substring(0, file.length() - 3);
                    try (InputStream is =
                            new GZIPInputStream(ClassLoaderUtils.getResourceAsStream(libPath))) {
                        Files.copy(is, tmp.resolve(f), StandardCopyOption.REPLACE_EXISTING);
                    }
                } else {
                    try (InputStream is = ClassLoaderUtils.getResourceAsStream(libPath)) {
                        Files.copy(is, tmp.resolve(file), StandardCopyOption.REPLACE_EXISTING);
                    }
                }
            }

            Utils.moveQuietly(tmp, dir);
            return path.toAbsolutePath().toString();
        } catch (IOException e) {
            throw new IllegalStateException("Failed to extract PaddlePaddle native library", e);
        } finally {
            if (tmp != null) {
                Utils.deleteQuietly(tmp);
            }
        }
    }

    private static String findLibraryInPath(String libPath) {
        String[] paths = libPath.split(File.pathSeparator);
        String mappedLibNames = System.mapLibraryName(NATIVE_LIB_NAME);

        for (String path : paths) {
            File p = new File(path);
            if (!p.exists()) {
                continue;
            }
            if (p.isFile() && p.getName().endsWith(mappedLibNames)) {
                return p.getAbsolutePath();
            }

            File file = new File(path, mappedLibNames);
            if (file.exists() && file.isFile()) {
                return file.getAbsolutePath();
            }
        }
        return null;
    }

    private static String downloadLibrary(Platform platform, AtomicBoolean fallback) {
        String version = platform.getVersion();
        String flavor = platform.getFlavor();
        String classifier = platform.getClassifier();
        String os = platform.getOsPrefix();

        String libName = System.mapLibraryName(NATIVE_LIB_NAME);
        Path cacheDir = Utils.getEngineCacheDir("paddle");
        Path dir = cacheDir.resolve(version + '-' + flavor + '-' + classifier);
        logger.debug("Using cache dir: {}", dir);
        Path path = dir.resolve(libName);
        if (Files.exists(path)) {
            return path.toAbsolutePath().toString();
        }

        Matcher matcher = VERSION_PATTERN.matcher(version);
        if (!matcher.matches()) {
            throw new IllegalArgumentException("Unexpected version: " + version);
        }

        Path tmp = null;
        String link = "https://publish.djl.ai/paddlepaddle-" + matcher.group(1);
        try (InputStream is = Utils.openUrl(link + "/files.txt")) {
            Files.createDirectories(cacheDir);

            List<String> lines = Utils.readLines(is);
            if (flavor.startsWith("cu")
                    && !lines.contains(flavor + '/' + os + "/native/lib/" + libName + ".gz")) {
                logger.warn("No matching cuda flavor for {} found: {}.", os, flavor);
                // fallback to CPU
                flavor = "cpu";
                fallback.set(true);
                // check again
                dir = cacheDir.resolve(version + '-' + flavor + '-' + classifier);
                path = dir.resolve(libName);
                if (Files.exists(path)) {
                    return path.toAbsolutePath().toString();
                }
            }

            tmp = Files.createTempDirectory(cacheDir, "tmp");
            boolean found = false;
            for (String line : lines) {
                if (line.startsWith(flavor + '/' + os + '/')) {
                    found = true;
                    URL url = new URL(link + '/' + line);
                    String fileName = line.substring(line.lastIndexOf('/') + 1, line.length() - 3);
                    logger.info("Downloading {} ...", url);
                    try (InputStream fis = new GZIPInputStream(Utils.openUrl(url))) {
                        Files.copy(fis, tmp.resolve(fileName), StandardCopyOption.REPLACE_EXISTING);
                    }
                }
            }
            if (!found) {
                throw new IllegalStateException(
                        "No PaddlePaddle native library matches your operating system: "
                                + platform);
            }

            Utils.moveQuietly(tmp, dir);
            return path.toAbsolutePath().toString();
        } catch (IOException e) {
            throw new IllegalStateException("Failed to download PaddlePaddle native library", e);
        } finally {
            if (tmp != null) {
                Utils.deleteQuietly(tmp);
            }
        }
    }
}
