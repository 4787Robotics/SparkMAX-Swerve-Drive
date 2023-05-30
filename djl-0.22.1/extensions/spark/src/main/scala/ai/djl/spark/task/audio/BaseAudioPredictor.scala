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
package ai.djl.spark.task.audio

import ai.djl.modality.audio.Audio
import ai.djl.spark.task.BasePredictor
import org.apache.spark.ml.util.Identifiable

/**
 * BaseAudioPredictor is the base class for audio predictors.
 *
 * @param uid An immutable unique ID for the object and its derivatives.
 */
abstract class BaseAudioPredictor[B](override val uid: String) extends BasePredictor[Audio, B] {

  def this() = this(Identifiable.randomUID("BaseAudioPredictor"))

  setDefault(inputClass, classOf[Audio])
}
