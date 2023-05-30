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
package ai.djl.repository;

import ai.djl.Application;
import ai.djl.repository.zoo.DefaultModelZoo;
import ai.djl.util.Progress;
import ai.djl.util.Utils;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.BufferedInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.URI;
import java.nio.file.Path;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * A {@code JarRepository} is a {@link Repository} contains an archive file from classpath.
 *
 * @see Repository
 */
public class JarRepository extends AbstractRepository {

    private static final Logger logger = LoggerFactory.getLogger(SimpleUrlRepository.class);

    private String artifactId;
    private String modelName;
    private String queryString;

    private Metadata metadata;
    private boolean resolved;

    JarRepository(String name, URI uri, String fileName, String queryString) {
        super(name, uri);
        this.queryString = queryString;
        modelName = arguments.get("model_name");
        artifactId = arguments.get("artifact_id");
        if (artifactId == null) {
            artifactId = fileName;
        }
        if (modelName == null) {
            modelName = artifactId;
        }
    }

    /** {@inheritDoc} */
    @Override
    public boolean isRemote() {
        return true;
    }

    /** {@inheritDoc} */
    @Override
    public Metadata locate(MRL mrl) {
        return getMetadata();
    }

    /** {@inheritDoc} */
    @Override
    public Artifact resolve(MRL mrl, Map<String, String> filter) {
        List<Artifact> artifacts = locate(mrl).getArtifacts();
        if (artifacts.isEmpty()) {
            return null;
        }
        return artifacts.get(0);
    }

    /** {@inheritDoc} */
    @Override
    public List<MRL> getResources() {
        Metadata m = getMetadata();
        if (m != null && !m.getArtifacts().isEmpty()) {
            MRL mrl = MRL.undefined(this, m.getGroupId(), m.getArtifactId());
            return Collections.singletonList(mrl);
        }
        return Collections.emptyList();
    }

    /** {@inheritDoc} */
    @Override
    protected void download(Path tmp, URI baseUri, Artifact.Item item, Progress progress)
            throws IOException {
        logger.debug("Extracting artifact: {} ...", uri);
        try (InputStream is = new BufferedInputStream(uri.toURL().openStream())) {
            save(is, tmp, item, progress);
        }
    }

    private synchronized Metadata getMetadata() {
        if (resolved) {
            return metadata;
        }

        resolved = true;
        Artifact artifact = new Artifact();
        artifact.setName(modelName);
        artifact.getArguments().putAll(arguments);
        Map<String, Artifact.Item> files = new ConcurrentHashMap<>();
        Artifact.Item item = new Artifact.Item();
        item.setUri(uri.getSchemeSpecificPart());
        item.setName(""); // avoid creating extra folder
        item.setArtifact(artifact);
        item.setSize(-1);
        files.put(artifactId, item);
        artifact.setFiles(files);

        metadata = new Metadata.MatchAllMetadata();
        metadata.setArtifactId(artifactId);
        metadata.setArtifacts(Collections.singletonList(artifact));
        String hash =
                Utils.hash(queryString == null ? uri.toString() : uri.toString() + queryString);
        MRL mrl = model(Application.UNDEFINED, DefaultModelZoo.GROUP_ID, hash);
        metadata.setRepositoryUri(mrl.toURI());

        return metadata;
    }
}
