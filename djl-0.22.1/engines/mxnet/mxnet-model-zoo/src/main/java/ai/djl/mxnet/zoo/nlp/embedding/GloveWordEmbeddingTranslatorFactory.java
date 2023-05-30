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
package ai.djl.mxnet.zoo.nlp.embedding;

import ai.djl.Model;
import ai.djl.ndarray.NDList;
import ai.djl.nn.core.Embedding;
import ai.djl.translate.ArgumentsUtil;
import ai.djl.translate.Translator;
import ai.djl.translate.TranslatorContext;
import ai.djl.translate.TranslatorFactory;
import ai.djl.util.Pair;

import java.lang.reflect.Type;
import java.util.Collections;
import java.util.Map;
import java.util.Set;

/** A {@link TranslatorFactory} that creates a {@link GloveWordEmbeddingTranslator} instance. */
public class GloveWordEmbeddingTranslatorFactory implements TranslatorFactory {

    /** {@inheritDoc} */
    @Override
    public Set<Pair<Type, Type>> getSupportedTypes() {
        return Collections.singleton(new Pair<>(String.class, NDList.class));
    }

    /** {@inheritDoc} */
    @Override
    @SuppressWarnings("unchecked")
    public <I, O> Translator<I, O> newInstance(
            Class<I> input, Class<O> output, Model model, Map<String, ?> arguments) {
        if (!isSupported(input, output)) {
            throw new IllegalArgumentException("Unsupported input/output types.");
        }
        String unknownToken = ArgumentsUtil.stringValue(arguments, "unknownToken");
        return (Translator<I, O>) new GloveWordEmbeddingTranslator(unknownToken);
    }

    private static final class GloveWordEmbeddingTranslator implements Translator<String, NDList> {

        private String unknownToken;
        private Embedding<String> embedding;

        public GloveWordEmbeddingTranslator(String unknownToken) {
            this.unknownToken = unknownToken;
        }

        /** {@inheritDoc} */
        @Override
        @SuppressWarnings("unchecked")
        public void prepare(TranslatorContext ctx) {
            try {
                embedding = (Embedding<String>) ctx.getBlock();
            } catch (ClassCastException e) {
                throw new IllegalArgumentException("The model was not an embedding", e);
            }
        }

        /** {@inheritDoc} */
        @Override
        public NDList processOutput(TranslatorContext ctx, NDList list) {
            return list;
        }

        /** {@inheritDoc} */
        @Override
        public NDList processInput(TranslatorContext ctx, String input) {
            if (embedding.hasItem(input)) {
                return new NDList(ctx.getNDManager().create(embedding.embed(input)));
            }
            return new NDList(ctx.getNDManager().create(embedding.embed(unknownToken)));
        }
    }
}
