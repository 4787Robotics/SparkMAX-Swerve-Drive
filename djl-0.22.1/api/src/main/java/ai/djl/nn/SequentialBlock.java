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
package ai.djl.nn;

import ai.djl.MalformedModelException;
import ai.djl.inference.streaming.StreamingBlock;
import ai.djl.ndarray.NDArray;
import ai.djl.ndarray.NDList;
import ai.djl.ndarray.NDManager;
import ai.djl.ndarray.types.DataType;
import ai.djl.ndarray.types.Shape;
import ai.djl.training.ParameterStore;
import ai.djl.util.PairList;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.function.Function;
import java.util.stream.Collectors;

/**
 * {@code SequentialBlock} is a {@link Block} whose children form a chain of blocks with each child
 * block feeding its output to the next. The output of the last child is returned as the output of
 * the {@code SequentialBlock}.
 *
 * <p>{@code SequentialBlock} has no direct parameters.
 */
public class SequentialBlock extends AbstractBlock implements StreamingBlock {

    private static final byte VERSION = 3;
    private boolean returnIntermediate;

    /**
     * Creates an empty sequential block. Use {@code add} and {@code addAll} to add blocks to be
     * executed in sequence.
     */
    public SequentialBlock() {
        super(VERSION);
    }

    /**
     * Adds an array of blocks to be executed in sequence, in order.
     *
     * @param blocks the array of blocks
     * @return this block
     */
    public SequentialBlock addAll(Block... blocks) {
        this.addAll(Arrays.asList(blocks));
        return this;
    }

    /**
     * Adds a {@link Collection} of blocks to be executed in sequence, in order.
     *
     * @param blocks the {@link Collection} of blocks
     * @return this block
     */
    public SequentialBlock addAll(Collection<Block> blocks) {
        blocks.forEach(this::add);
        return this;
    }

    /**
     * Adds the given {@link Block} to the block to be executed in order.
     *
     * @param block the block to be added to the sequence of blocks
     * @return this block
     */
    public SequentialBlock add(Block block) {
        if (block != null) {
            addChildBlock(block.getClass().getSimpleName(), block);
        }
        return this;
    }

    /**
     * Adds a {@link LambdaBlock} that applies the given function to the sequence of blocks.
     *
     * @param f the function forms the {@link LambdaBlock}
     * @return this block
     */
    public SequentialBlock add(Function<NDList, NDList> f) {
        add(new LambdaBlock(f));
        return this;
    }

    /**
     * Adds a {@link LambdaBlock} that applies the given function to the sequence of blocks.
     *
     * @param f the function forms the {@link LambdaBlock}
     * @param name the function name
     * @return this block
     */
    public SequentialBlock add(Function<NDList, NDList> f, String name) {
        add(new LambdaBlock(f, name));
        return this;
    }

    /**
     * Adds a {@link LambdaBlock#singleton(Function)} that applies the given function to the
     * sequence of blocks.
     *
     * @param f the function forms the {@link LambdaBlock}
     * @return this block
     * @see LambdaBlock#singleton(Function)
     */
    public SequentialBlock addSingleton(Function<NDArray, NDArray> f) {
        add(LambdaBlock.singleton(f));
        return this;
    }

    /**
     * Adds a {@link LambdaBlock#singleton(Function)} that applies the given function to the
     * sequence of blocks.
     *
     * @param f the function forms the {@link LambdaBlock}
     * @param name the function name
     * @return this block
     * @see LambdaBlock#singleton(Function)
     */
    public SequentialBlock addSingleton(Function<NDArray, NDArray> f, String name) {
        add(LambdaBlock.singleton(f, name));
        return this;
    }

    /** Removes the {@link Block} added last from the sequence of blocks. */
    public void removeLastBlock() {
        children.remove(children.size() - 1);
    }

    /**
     * Replaces the {@link Block} last added from the sequence of blocks, and adds the given block.
     *
     * @param block the block to replace the last block with
     */
    public void replaceLastBlock(Block block) {
        removeLastBlock();
        if (block != null) {
            add(block);
        }
    }

    /**
     * Returns whether the block returns all intermediate block results or only the end of the
     * sequential chain.
     *
     * @return whether the block returns all intermediate block results or only the end of the
     *     sequential chain
     */
    public boolean isReturnIntermediate() {
        return returnIntermediate;
    }

    /**
     * Sets whether the block returns all intermediate sequence results.
     *
     * @param returnIntermediate true for intermediates, false for only chain result (default and
     *     typical behavior is false)
     * @return this {@link SequentialBlock}
     */
    public SequentialBlock setReturnIntermediate(boolean returnIntermediate) {
        this.returnIntermediate = returnIntermediate;
        return this;
    }

    /** {@inheritDoc} */
    @Override
    protected NDList forwardInternal(
            ParameterStore parameterStore,
            NDList inputs,
            boolean training,
            PairList<String, Object> params) {
        List<NDList> past = new ArrayList<>(children.size());
        NDList current = inputs;
        for (Block block : children.values()) {
            current = block.forward(parameterStore, current, training);
            past.add(current);
        }
        if (returnIntermediate) {
            return new NDList(
                    past.stream().flatMap(Collection::stream).collect(Collectors.toList()));
        }
        return current;
    }

    /** {@inheritDoc} */
    @Override
    protected NDList forwardInternal(
            ParameterStore parameterStore,
            NDList data,
            NDList labels,
            PairList<String, Object> params) {
        List<NDList> past = new ArrayList<>(children.size());
        NDList current = data;
        for (Block block : children.values()) {
            current = block.forward(parameterStore, current, labels, params);
            past.add(current);
        }
        if (returnIntermediate) {
            return new NDList(
                    past.stream().flatMap(Collection::stream).collect(Collectors.toList()));
        }
        return current;
    }

    /** {@inheritDoc} */
    @Override
    public Iterator<NDList> forwardStreamIter(
            ParameterStore parameterStore,
            NDList inputs,
            boolean training,
            PairList<String, Object> params) {
        return new StreamIterator(parameterStore, inputs, training);
    }

    /** {@inheritDoc} */
    @Override
    public void initializeChildBlocks(NDManager manager, DataType dataType, Shape... inputShapes) {
        Shape[] shapes = inputShapes;
        DataType[] lastDataTypes = null;
        for (Block child : getChildren().values()) {
            child.initialize(manager, dataType, shapes);
            shapes = child.getOutputShapes(shapes, lastDataTypes);
            lastDataTypes = child.getOutputDataTypes();
        }
    }

    /** {@inheritDoc} */
    @Override
    public Shape[] getOutputShapes(Shape[] inputs) {
        if (children.isEmpty()) {
            throw new IllegalArgumentException("The sequential block is empty");
        }
        List<Shape[]> past = new ArrayList<>(children.size());
        Shape[] current = inputs;
        for (Block block : children.values()) {
            current = block.getOutputShapes(current);
            past.add(current);
        }
        if (returnIntermediate) {
            return past.stream().flatMap(Arrays::stream).toArray(Shape[]::new);
        }
        return current;
    }

    /** {@inheritDoc} */
    @Override
    protected void saveMetadata(DataOutputStream os) throws IOException {
        saveInputShapes(os);
        os.writeBoolean(returnIntermediate);
    }

    /** {@inheritDoc} */
    @Override
    public void loadMetadata(byte loadVersion, DataInputStream is)
            throws IOException, MalformedModelException {
        if (loadVersion == version) {
            readInputShapes(is);
            returnIntermediate = is.readBoolean();
        } else if (loadVersion == 2) {
            readInputShapes(is);
        } else {
            throw new MalformedModelException("Unsupported encoding version: " + loadVersion);
        }
    }

    private final class StreamIterator implements Iterator<NDList> {

        private int childIndex;
        private ParameterStore parameterStore;
        private NDList current;
        private boolean training;

        private StreamIterator(ParameterStore parameterStore, NDList inputs, boolean training) {
            this.parameterStore = parameterStore;
            this.current = inputs;
            this.training = training;
            childIndex = 0;
        }

        /** {@inheritDoc} */
        @Override
        public boolean hasNext() {
            return childIndex < children.size();
        }

        /** {@inheritDoc} */
        @Override
        public NDList next() {
            current =
                    children.get(childIndex++)
                            .getValue()
                            .forward(parameterStore, current, training);
            return current;
        }
    }
}
