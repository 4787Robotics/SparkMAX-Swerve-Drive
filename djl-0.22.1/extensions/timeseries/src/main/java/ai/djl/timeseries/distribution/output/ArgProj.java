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

package ai.djl.timeseries.distribution.output;

import ai.djl.ndarray.NDList;
import ai.djl.ndarray.NDManager;
import ai.djl.ndarray.types.DataType;
import ai.djl.ndarray.types.Shape;
import ai.djl.nn.AbstractBlock;
import ai.djl.nn.Block;
import ai.djl.nn.core.Linear;
import ai.djl.training.ParameterStore;
import ai.djl.util.Pair;
import ai.djl.util.PairList;
import ai.djl.util.Preconditions;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

/**
 * A Block used to map the output of a dense layer to statistical parameters, like mean and standard
 * deviation. It will be used in both training and inference.
 */
public final class ArgProj extends AbstractBlock {

    private Block domainMap;
    private List<Block> proj;

    ArgProj(Builder builder) {
        proj = new ArrayList<>();
        for (Pair<String, Integer> entry : builder.argsDim) {
            proj.add(
                    addChildBlock(
                            String.format("%s_distr_%s", builder.prefix, entry.getKey()),
                            Linear.builder().setUnits(entry.getValue()).build()));
        }
        domainMap =
                addChildBlock(String.format("%s_domain_map", builder.prefix), builder.domainMap);
    }

    /** {@inheritDoc} */
    @Override
    protected void initializeChildBlocks(
            NDManager manager, DataType dataType, Shape... inputShapes) {
        for (Block block : proj) {
            block.initialize(manager, dataType, inputShapes);
        }
    }

    /** {@inheritDoc} */
    @Override
    protected NDList forwardInternal(
            ParameterStore parameterStore,
            NDList inputs,
            boolean training,
            PairList<String, Object> params) {
        NDList paramsUnbounded = new NDList();
        for (Block block : proj) {
            paramsUnbounded.add(
                    block.forward(parameterStore, inputs, training, params).singletonOrThrow());
        }
        return domainMap.forward(parameterStore, paramsUnbounded, training, params);
    }

    /** {@inheritDoc} */
    @Override
    public Shape[] getOutputShapes(Shape[] inputShapes) {
        Shape[] projOutShapes = new Shape[proj.size()];
        for (int i = 0; i < proj.size(); i++) {
            projOutShapes[i] = proj.get(i).getOutputShapes(inputShapes)[0];
        }
        return domainMap.getOutputShapes(projOutShapes);
    }

    /**
     * Creates a builder to build a {@code ArgProj}.
     *
     * @return a new builder
     */
    public static Builder builder() {
        return new Builder();
    }

    /** The Builder to construct a {@code ArgProj} type of {@link Block}. */
    public static final class Builder {
        private PairList<String, Integer> argsDim;
        private Function<NDList, NDList> domainMap;
        private String prefix = "";

        /**
         * Set the arguments dimensions of distribution.
         *
         * @param argsDim the arguments dimension
         * @return this builder
         */
        public Builder setArgsDim(PairList<String, Integer> argsDim) {
            this.argsDim = argsDim;
            return this;
        }

        /**
         * Set the domain map function.
         *
         * @param domainMap the domain map function
         * @return this builder
         */
        public Builder setDomainMap(Function<NDList, NDList> domainMap) {
            this.domainMap = domainMap;
            return this;
        }

        /**
         * Set the block name prefix.
         *
         * @param prefix the prefix
         * @return this builder
         */
        public Builder optPrefix(String prefix) {
            this.prefix = prefix;
            return this;
        }

        /**
         * Build a {@link ArgProj} block.
         *
         * @return the {@link ArgProj} block.
         */
        public ArgProj build() {
            Preconditions.checkArgument(argsDim != null, "must specify dim args");
            Preconditions.checkArgument(domainMap != null, "must specify domain PairList function");
            return new ArgProj(this);
        }
    }
}
