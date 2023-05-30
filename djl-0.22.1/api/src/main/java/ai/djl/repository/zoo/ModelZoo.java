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
package ai.djl.repository.zoo;

import ai.djl.Application;
import ai.djl.MalformedModelException;
import ai.djl.repository.Artifact;
import ai.djl.repository.MRL;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.ServiceLoader;
import java.util.Set;
import java.util.TreeMap;
import java.util.concurrent.ConcurrentHashMap;

/** An interface represents a collection of models. */
public abstract class ModelZoo {

    private static final Map<String, ModelZoo> MODEL_ZOO_MAP = new ConcurrentHashMap<>();

    private Map<String, ModelLoader> modelLoaders = new ConcurrentHashMap<>();

    static {
        ServiceLoader<ZooProvider> providers = ServiceLoader.load(ZooProvider.class);
        for (ZooProvider provider : providers) {
            registerModelZoo(provider);
        }
    }

    /**
     * Returns the global unique identifier of the {@code ModelZoo}.
     *
     * <p>We recommend to use reverse DNS name as your model zoo group ID to make sure it's not
     * conflict with other ModelZoos.
     *
     * @return the global unique identifier of the {@code ModelZoo}
     */
    public abstract String getGroupId();

    /**
     * Lists the available model families in the ModelZoo.
     *
     * @return the list of all available model families
     */
    public Collection<ModelLoader> getModelLoaders() {
        return modelLoaders.values();
    }

    /**
     * Returns the {@link ModelLoader} based on the model name.
     *
     * @param name the name of the model
     * @return the {@link ModelLoader} of the model
     */
    public ModelLoader getModelLoader(String name) {
        return modelLoaders.get(name);
    }

    /**
     * Returns all supported engine names.
     *
     * @return all supported engine names
     */
    public abstract Set<String> getSupportedEngines();

    protected final void addModel(MRL mrl) {
        modelLoaders.put(mrl.getArtifactId(), new BaseModelLoader(mrl));
    }

    protected final void addModel(ModelLoader loader) {
        modelLoaders.put(loader.getArtifactId(), loader);
    }

    /**
     * Refreshes model zoo.
     *
     * @param provider the {@code ZooProvider}
     */
    public static void registerModelZoo(ZooProvider provider) {
        ModelZoo zoo = provider.getModelZoo();
        MODEL_ZOO_MAP.put(zoo.getGroupId(), zoo);
    }

    /**
     * Returns available model zoos.
     *
     * @return a list of model zoo
     */
    public static Collection<ModelZoo> listModelZoo() {
        return MODEL_ZOO_MAP.values();
    }

    /**
     * Returns the {@code ModelZoo} with the {@code groupId}.
     *
     * @param groupId the model zoo group id to check for
     * @return the {@code ModelZoo} with the {@code groupId}
     */
    public static ModelZoo getModelZoo(String groupId) {
        return MODEL_ZOO_MAP.get(groupId);
    }

    /**
     * Returns whether a model zoo with the group id is available.
     *
     * @param groupId the model zoo group id to check for
     * @return whether a model zoo with the group id is available
     */
    public static boolean hasModelZoo(String groupId) {
        return MODEL_ZOO_MAP.containsKey(groupId);
    }

    /**
     * Load the {@link ZooModel} that matches this criteria.
     *
     * @param criteria the requirements for the model
     * @param <I> the input data type for preprocessing
     * @param <O> the output data type after postprocessing
     * @return the model that matches the criteria
     * @throws IOException for various exceptions loading data from the repository
     * @throws ModelNotFoundException if no model with the specified criteria is found
     * @throws MalformedModelException if the model data is malformed
     */
    public static <I, O> ZooModel<I, O> loadModel(Criteria<I, O> criteria)
            throws IOException, ModelNotFoundException, MalformedModelException {
        return criteria.loadModel();
    }

    /**
     * Returns the available {@link Application} and their model artifact metadata.
     *
     * @return the available {@link Application} and their model artifact metadata
     * @throws IOException if failed to download to repository metadata
     * @throws ModelNotFoundException if failed to parse repository metadata
     */
    public static Map<Application, List<Artifact>> listModels()
            throws IOException, ModelNotFoundException {
        return listModels(Criteria.builder().build());
    }

    /**
     * Returns the available {@link Application} and their model artifact metadata.
     *
     * @param criteria the requirements for the model
     * @return the available {@link Application} and their model artifact metadata
     * @throws IOException if failed to download to repository metadata
     * @throws ModelNotFoundException if failed to parse repository metadata
     */
    public static Map<Application, List<Artifact>> listModels(Criteria<?, ?> criteria)
            throws IOException, ModelNotFoundException {
        String artifactId = criteria.getArtifactId();
        ModelZoo modelZoo = criteria.getModelZoo();
        String groupId = criteria.getGroupId();
        String engine = criteria.getEngine();
        Application application = criteria.getApplication();

        @SuppressWarnings("PMD.UseConcurrentHashMap")
        Map<Application, List<Artifact>> models =
                new TreeMap<>(Comparator.comparing(Application::getPath));
        for (ModelZoo zoo : listModelZoo()) {
            if (modelZoo != null) {
                if (groupId != null && !modelZoo.getGroupId().equals(groupId)) {
                    continue;
                }
                Set<String> supportedEngine = modelZoo.getSupportedEngines();
                if (engine != null && !supportedEngine.contains(engine)) {
                    continue;
                }
            }
            for (ModelLoader loader : zoo.getModelLoaders()) {
                Application app = loader.getApplication();
                String loaderArtifactId = loader.getArtifactId();
                if (artifactId != null && !artifactId.equals(loaderArtifactId)) {
                    // filter out by model loader artifactId
                    continue;
                }
                if (application != Application.UNDEFINED
                        && app != Application.UNDEFINED
                        && !app.matches(application)) {
                    // filter out ModelLoader by application
                    continue;
                }
                final List<Artifact> artifacts = loader.listModels();
                models.compute(
                        app,
                        (key, val) -> {
                            if (val == null) {
                                val = new ArrayList<>();
                            }
                            val.addAll(artifacts);
                            return val;
                        });
            }
        }
        return models;
    }
}
