/*
 * Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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
package ai.djl.mxnet.jna;

import ai.djl.util.ClassLoaderUtils;
import ai.djl.util.Platform;
import ai.djl.util.Utils;

import com.sun.jna.Library;
import com.sun.jna.Native;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.zip.GZIPInputStream;

/**
 * Utilities for finding the MXNet Engine binary on the System.
 *
 * <p>The Engine will be searched for in a variety of locations in the following order:
 *
 * <ol>
 *   <li>In the path specified by the MXNET_LIBRARY_PATH environment variable
 *   <li>In a jar file location in the classpath. These jars can be created with the mxnet-native
 *       module.
 *   <li>In the python3 path. These can be installed using pip.
 *   <li>In the python path. These can be installed using pip.
 * </ol>
 */
@SuppressWarnings("MissingJavadocMethod")
public final class LibUtils {

    private static final Logger logger = LoggerFactory.getLogger(LibUtils.class);

    private static final String LIB_NAME = "mxnet";

    private static final Pattern VERSION_PATTERN =
            Pattern.compile("(\\d+\\.\\d+\\.\\d+(-[a-z]+)?)(-SNAPSHOT)?(-\\d+)?");

    private LibUtils() {}

    public static MxnetLibrary loadLibrary() {
        String libName = getLibName();
        logger.debug("Loading mxnet library from: {}", libName);

        if (System.getProperty("os.name").startsWith("Linux")) {
            Map<String, Integer> options = new ConcurrentHashMap<>();
            int rtld = 1; // Linux RTLD lazy + local
            options.put(Library.OPTION_OPEN_FLAGS, rtld);
            return Native.load(libName, MxnetLibrary.class, options);
        }

        return Native.load(libName, MxnetLibrary.class);
    }

    public static String getLibName() {
        String libName = LibUtils.findOverrideLibrary();
        if (libName == null) {
            libName = LibUtils.findLibraryInClasspath();
        }
        return libName;
    }

    private static String findOverrideLibrary() {
        String libPath = Utils.getEnvOrSystemProperty("MXNET_LIBRARY_PATH");
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

    private static synchronized String findLibraryInClasspath() {
        String overrideVersion = Utils.getEnvOrSystemProperty("MXNET_VERSION");
        if (overrideVersion != null) {
            Platform platform = Platform.detectPlatform("mxnet", overrideVersion);
            return downloadMxnet(platform);
        }

        Platform platform = Platform.detectPlatform("mxnet");
        if (platform.isPlaceholder()) {
            return downloadMxnet(platform);
        }
        return loadLibraryFromClasspath(platform);
    }

    private static String loadLibraryFromClasspath(Platform platform) {
        Path tmp = null;
        try {
            String libName = System.mapLibraryName(LIB_NAME);
            Path cacheFolder = Utils.getEngineCacheDir("mxnet");
            String version = platform.getVersion();
            String flavor = platform.getFlavor();
            if ("cpu".equals(flavor)) {
                flavor = "mkl";
            } else if (!flavor.endsWith("mkl")) {
                flavor += "mkl"; // NOPMD
            }
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
                logger.info("Extracting {} to cache ...", libPath);
                try (InputStream is = ClassLoaderUtils.getResourceAsStream(libPath)) {
                    Files.copy(is, tmp.resolve(file), StandardCopyOption.REPLACE_EXISTING);
                }
            }

            Utils.moveQuietly(tmp, dir);
            return path.toAbsolutePath().toString();
        } catch (IOException e) {
            throw new IllegalStateException("Failed to extract MXNet native library", e);
        } finally {
            if (tmp != null) {
                Utils.deleteQuietly(tmp);
            }
        }
    }

    private static String findLibraryInPath(String libPath) {
        String[] paths = libPath.split(File.pathSeparator);
        List<String> mappedLibNames;
        if (com.sun.jna.Platform.isMac()) {
            mappedLibNames = Arrays.asList("libmxnet.dylib", "libmxnet.jnilib", "libmxnet.so");
        } else {
            mappedLibNames = Collections.singletonList(System.mapLibraryName(LIB_NAME));
        }

        for (String path : paths) {
            File p = new File(path);
            if (!p.exists()) {
                continue;
            }
            for (String name : mappedLibNames) {
                if (p.isFile() && p.getName().endsWith(name)) {
                    return p.getAbsolutePath();
                }

                File file = new File(path, name);
                if (file.exists() && file.isFile()) {
                    return file.getAbsolutePath();
                }
            }
        }
        return null;
    }

    private static String downloadMxnet(Platform platform) {
        String version = platform.getVersion();
        String flavor = platform.getFlavor();
        if ("cpu".equals(flavor)) {
            flavor = "mkl";
        } else if (!flavor.endsWith("mkl")) {
            flavor += "mkl"; // NOPMD
        }
        String classifier = platform.getClassifier();
        String cudaArch = platform.getCudaArch();
        String os = platform.getOsPrefix();

        String libName = System.mapLibraryName(LIB_NAME);
        Path cacheFolder = Utils.getEngineCacheDir("mxnet");
        Path dir = cacheFolder.resolve(version + '-' + flavor + '-' + classifier);
        Path path = dir.resolve(libName);
        if (Files.exists(path)) {
            logger.debug("Using cache dir: {}", dir);
            return path.toAbsolutePath().toString();
        }

        Matcher matcher = VERSION_PATTERN.matcher(version);
        if (!matcher.matches()) {
            throw new IllegalArgumentException("Unexpected version: " + version);
        }

        Path tmp = null;
        String link = "https://publish.djl.ai/mxnet-" + matcher.group(1);
        try (InputStream is = Utils.openUrl(link + "/files.txt")) {
            Files.createDirectories(cacheFolder);
            tmp = Files.createTempDirectory(cacheFolder, "tmp");

            List<String> lines = Utils.readLines(is);
            if (cudaArch != null) {
                // has CUDA
                if ("win".equals(os)) {
                    if (!lines.contains(os + '/' + flavor + "/mxnet_" + cudaArch + ".dll.gz")) {
                        logger.warn(
                                "No matching cuda flavor for {} found: {}/sm_{}.",
                                os,
                                flavor,
                                cudaArch);
                        // fallback to CPU
                        flavor = "mkl";
                    }
                } else if ("linux".equals(os)) {
                    // MXNet must use exactly matched cuda minor version
                    if (!lines.contains(os + '/' + flavor + "/libmxnet.so.gz")
                            || !supported(platform)) {
                        logger.warn(
                                "No matching cuda flavor for {} found: {}/sm_{}.",
                                os,
                                flavor,
                                cudaArch);
                        // fallback to CPU
                        flavor = "mkl";
                    }
                } else {
                    throw new AssertionError("Unsupported GPU operating system: " + os);
                }

                // check again in case fallback to cpu or different cuda minor version
                dir = cacheFolder.resolve(version + '-' + flavor + '-' + classifier);
                path = dir.resolve(libName);
                if (Files.exists(path)) {
                    return path.toAbsolutePath().toString();
                }
            }

            logger.debug("Using cache dir: {}", dir);

            boolean found = false;
            for (String line : lines) {
                if (line.startsWith(os + "/common/") || line.startsWith(os + '/' + flavor + '/')) {
                    found = true;
                    URL url = new URL(link + '/' + line);
                    String fileName = line.substring(line.lastIndexOf('/') + 1, line.length() - 3);
                    if ("win".equals(os)) {
                        if ("libmxnet.dll".equals(fileName)) {
                            fileName = "mxnet.dll";
                        } else if (fileName.startsWith("mxnet_")) {
                            if (!("mxnet_" + cudaArch + ".dll").equals(fileName)) {
                                continue;
                            }
                            fileName = "mxnet.dll"; // split CUDA build
                        }
                    }
                    logger.info("Downloading {} ...", fileName);
                    try (InputStream fis = new GZIPInputStream(Utils.openUrl(url))) {
                        Files.copy(fis, tmp.resolve(fileName), StandardCopyOption.REPLACE_EXISTING);
                    }
                }
            }
            if (!found) {
                throw new IllegalStateException(
                        "No MXNet native library matches your operating system: " + platform);
            }

            Utils.moveQuietly(tmp, dir);
            return path.toAbsolutePath().toString();
        } catch (IOException e) {
            throw new IllegalStateException("Failed to download MXNet native library", e);
        } finally {
            if (tmp != null) {
                Utils.deleteQuietly(tmp);
            }
        }
    }

    private static boolean supported(Platform platform) {
        // mxnet-native-cu102mkl:1.8.0: 3.0, 5.0, 6.0, 7.0, 7.5
        // mxnet-native-cu110mkl:1.8.0: 5.0, 6.0, 7.0, 8.0
        // mxnet-native-cu112mkl:1.9.1: 5.0, 6.0, 7.0, 7.5, 8.0, 8.6
        String version = platform.getVersion();
        if (version.startsWith("1.8.") || version.startsWith("1.9.")) {
            String flavor = platform.getFlavor();
            String cudaArch = platform.getCudaArch();
            if (flavor.startsWith("cu11")) {
                if (version.startsWith("1.8.")) {
                    return Arrays.asList("50", "60", "70", "80").contains(cudaArch);
                }
                return Arrays.asList("50", "60", "70", "75", "80", "86").contains(cudaArch);
            } else if (flavor.startsWith("cu10")) {
                return Arrays.asList("30", "50", "60", "70", "75").contains(cudaArch);
            }
        }
        return true;
    }
}
