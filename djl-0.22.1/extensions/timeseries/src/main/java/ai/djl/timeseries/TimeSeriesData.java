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
package ai.djl.timeseries;

import ai.djl.ndarray.NDArray;
import ai.djl.ndarray.NDList;
import ai.djl.timeseries.dataset.FieldName;
import ai.djl.util.Pair;
import ai.djl.util.PairList;

import java.time.LocalDateTime;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * {@link TimeSeriesData} is a DataEntry for managing time series data in preprocess. It contains a
 * key-values entries mapping the {@link FieldName} and {@link NDArray} to generate the time
 * features.
 *
 * <p>This class provides a convenient way for user to featurize the data.
 */
public class TimeSeriesData extends PairList<String, NDArray> {

    private LocalDateTime startTime;
    private LocalDateTime forecastStartTime;

    /**
     * Constructs an empty {@code TimeSeriesData} with the specified initial capacity.
     *
     * @param initialCapacity the initial capacity of the list
     * @throws IllegalArgumentException if the specified initial capacity is negative
     */
    public TimeSeriesData(int initialCapacity) {
        super(initialCapacity);
    }

    /**
     * Constructs a {@code TimeSeriesData} containing the elements of the specified keys and values.
     *
     * @param keys the key list containing elements to be placed into this PairList
     * @param values the value list containing elements to be placed into this PairList
     * @throws IllegalArgumentException if the keys and values size are different
     */
    public TimeSeriesData(List<String> keys, List<NDArray> values) {
        super(keys, values);
    }

    /**
     * Constructs a {@code TimeSeriesData} containing the elements of the specified list of Pairs.
     *
     * @param list the list containing elements to be placed into this PairList
     */
    public TimeSeriesData(List<Pair<String, NDArray>> list) {
        super(list);
    }

    /**
     * Constructs a {@code TimeSeriesData} containing the elements of the specified map.
     *
     * @param map the map contains keys and values
     */
    public TimeSeriesData(Map<String, NDArray> map) {
        super(map);
    }

    /**
     * Constructs a {@link NDList} containing the remaining {@link NDArray} for {@link FieldName}.
     *
     * @return a {@link NDList}
     */
    public NDList toNDList() {
        List<NDArray> arrays = this.values();
        int index = 0;
        for (NDArray array : arrays) {
            array.setName("data" + index);
            index++;
        }
        return new NDList(arrays);
    }

    /**
     * Returns the time series start time.
     *
     * @return a {@link LocalDateTime} representing start time
     */
    public LocalDateTime getStartTime() {
        return startTime;
    }

    /**
     * Returns the time series forecasting time.
     *
     * @return a {@link LocalDateTime} representing the time to forecast
     */
    public LocalDateTime getForecastStartTime() {
        return forecastStartTime;
    }

    /**
     * Adds a fieldName and value to the list.
     *
     * @param fieldName the {@link FieldName}.
     * @param value the {@link NDArray} value
     */
    public void add(FieldName fieldName, NDArray value) {
        add(fieldName.name(), value);
    }

    /**
     * Returns the value for the fieldName.
     *
     * @param fieldName the {@link FieldName} of the element to get
     * @return the {@link NDArray} value for the {@link FieldName}
     */
    public NDArray get(FieldName fieldName) {
        return get(fieldName.name());
    }

    /**
     * Replaces the existing {@link NDArray} of {@link FieldName} to the value.
     *
     * @param fieldName the {@link FieldName}
     * @param value the {@link NDArray} value
     */
    public void setField(String fieldName, NDArray value) {
        remove(fieldName);
        add(fieldName, value);
    }

    /**
     * Replace the existing {@link NDArray} of {@link FieldName} to the value.
     *
     * @param fieldName the {@link FieldName}.
     * @param value the {@link NDArray} value.
     */
    public void setField(FieldName fieldName, NDArray value) {
        setField(fieldName.name(), value);
    }

    /**
     * Set the time series start time.
     *
     * @param value the {@link LocalDateTime} start time.
     */
    public void setStartTime(LocalDateTime value) {
        this.startTime = value;
    }

    /**
     * Set the time series forecasting time.
     *
     * @param value the {@link LocalDateTime} time to forecast
     */
    public void setForecastStartTime(LocalDateTime value) {
        this.forecastStartTime = value;
    }

    /**
     * Removes the key-value pair for the {@link FieldName}.
     *
     * @param fieldName the {@link FieldName} to be removed.
     */
    public void remove(FieldName fieldName) {
        remove(fieldName.name());
    }

    /** {@inheritDoc} * */
    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (o == null || getClass() != o.getClass()) {
            return false;
        }
        if (!super.equals(o)) {
            return false;
        }
        TimeSeriesData pairs = (TimeSeriesData) o;
        return Objects.equals(startTime, pairs.startTime)
                && Objects.equals(forecastStartTime, pairs.forecastStartTime);
    }

    /** {@inheritDoc} * */
    @Override
    public int hashCode() {
        return Objects.hash(super.hashCode(), startTime, forecastStartTime);
    }
}
