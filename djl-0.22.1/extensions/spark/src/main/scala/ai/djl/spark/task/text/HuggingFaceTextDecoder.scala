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
package ai.djl.spark.task.text

import ai.djl.huggingface.tokenizers.HuggingFaceTokenizer
import org.apache.spark.ml.param.Param
import org.apache.spark.ml.param.shared.{HasInputCol, HasOutputCol}
import org.apache.spark.ml.util.Identifiable
import org.apache.spark.sql.types.{ArrayType, LongType, StringType, StructField, StructType}
import org.apache.spark.sql.{DataFrame, Dataset, Row}

/**
 * TextDecoder performs text decoding using HuggingFace tokenizers in Spark.
 *
 * @param uid An immutable unique ID for the object and its derivatives.
 */
class HuggingFaceTextDecoder(override val uid: String) extends BaseTextPredictor[Array[Long], String]
  with HasInputCol with HasOutputCol {

  def this() = this(Identifiable.randomUID("HuggingFaceTextDecoder"))

  final val tokenizer = new Param[String](this, "tokenizer", "The name of the tokenizer")

  private var inputColIndex : Int = _

  /**
   * Sets the inputCol parameter.
   *
   * @param value the value of the parameter
   */
  def setInputCol(value: String): this.type = set(inputCol, value)

  /**
   * Sets the outputCol parameter.
   *
   * @param value the value of the parameter
   */
  def setOutputCol(value: String): this.type = set(outputCol, value)

  /**
   * Sets the tokenizer parameter.
   *
   * @param value the value of the parameter
   */
  def setTokenizer(value: String): this.type = set(tokenizer, value)

  setDefault(inputClass, classOf[Array[Long]])
  setDefault(outputClass, classOf[String])
  setDefault(translatorFactory, null)

  /**
   * Decodes String from the input ids on the provided dataset.
   *
   * @param dataset input dataset
   * @return output dataset
   */
  def decode(dataset: Dataset[_]): DataFrame = {
    transform(dataset)
  }

  /** @inheritdoc */
  override def transform(dataset: Dataset[_]): DataFrame = {
    inputColIndex = dataset.schema.fieldIndex($(inputCol))
    super.transform(dataset)
  }

  /** @inheritdoc */
  override def transformRows(iter: Iterator[Row]): Iterator[Row] = {
    val t = HuggingFaceTokenizer.newInstance($(tokenizer))
    iter.map(row => {
      Row.fromSeq(row.toSeq :+ t.decode(row.getAs[Seq[Long]]($(inputCol)).toArray))
    })
  }

  /** @inheritdoc */
  def validateInputType(schema: StructType): Unit = {
    validateType(schema($(inputCol)), ArrayType(LongType))
  }

  /** @inheritdoc */
  override def transformSchema(schema: StructType): StructType = {
    val outputSchema = StructType(schema.fields :+ StructField($(outputCol), StringType))
    outputSchema
  }
}
