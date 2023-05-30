#!/usr/bin/env python
#
# Copyright 2022 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License"). You may not use this file
# except in compliance with the License. A copy of the License is located at
#
# http://aws.amazon.com/apache2.0/
#
# or in the "LICENSE.txt" file accompanying this file. This file is distributed on an "AS IS"
# BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, express or implied. See the License for
# the specific language governing permissions and limitations under the License.
import logging
import shutil
import sys

from arg_parser import converter_args
from fill_mask_converter import FillMaskConverter
from huggingface_models import HuggingfaceModels
from question_answering_converter import QuestionAnsweringConverter
from sentence_similarity_converter import SentenceSimilarityConverter
from text_classification_converter import TextClassificationConverter
from token_classification_converter import TokenClassificationConverter

SUPPORTED_TASK = {
    "fill-mask": FillMaskConverter(),
    "question-answering": QuestionAnsweringConverter(),
    "sentence-similarity": SentenceSimilarityConverter(),
    "text-classification": TextClassificationConverter(),
    "token-classification": TokenClassificationConverter(),
}


def main():
    logging.basicConfig(stream=sys.stdout,
                        format="%(message)s",
                        level=logging.INFO)
    args = converter_args()

    huggingface_models = HuggingfaceModels(args.output_dir)
    temp_dir = f"{args.output_dir}/tmp"

    models = huggingface_models.list_models(args)

    for model in models:
        task = model["task"]
        model_info = model["model_info"]
        converter = SUPPORTED_TASK[task]

        result, reason, size = converter.save_model(model_info, args, temp_dir)
        if not result:
            logging.error(f"{model_info.modelId}: {reason}")

        huggingface_models.update_progress(model_info, converter.application,
                                           result, reason, size, args.cpu_only)
        shutil.rmtree(temp_dir)

    logging.info("finished.")


if __name__ == "__main__":
    main()
