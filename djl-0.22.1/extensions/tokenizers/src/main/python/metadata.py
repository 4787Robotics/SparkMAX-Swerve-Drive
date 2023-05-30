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
import json
from huggingface_models import get_lang_tags


class HuggingfaceMetadata:

    def __init__(self, model_info, application: str, translator: str,
                 sha1: str, file_size: int):
        self.model_info = model_info
        self.artifact_id = model_info.modelId
        self.model_name = model_info.modelId.split("/")[-1]
        self.application = application
        self.translator = translator
        self.sha1 = sha1
        self.file_size = file_size

    def save_metadata(self, metadata_file: str):
        properties = get_lang_tags(self.model_info)

        metadata = {
            "metadataVersion":
            "0.2",
            "resourceType":
            "model",
            "application":
            self.application,
            "groupId":
            "ai.djl.huggingface.pytorch",
            "artifactId":
            self.artifact_id,
            "name":
            self.model_name,
            "description":
            f"Huggingface transformers model: {self.model_name}",
            "website":
            "http://www.djl.ai/extensions/tokenizers",
            "licenses": {
                "license": {
                    "name": "The Apache License, Version 2.0",
                    "url": "https://www.apache.org/licenses/LICENSE-2.0"
                }
            },
            "artifacts": [{
                "version": "0.0.1",
                "snapshot": False,
                "name": self.model_name,
                "properties": properties,
                "arguments": {
                    "engine": "PyTorch",
                    "translatorFactory": self.translator
                },
                "options": {
                    "mapLocation": True
                },
                "files": {
                    "model": {
                        "uri": f"0.0.1/{self.model_name}.zip",
                        "name": "",
                        "sha1Hash": self.sha1,
                        "size": self.file_size
                    }
                }
            }]
        }
        with open(metadata_file, 'w') as f:
            json.dump(metadata,
                      f,
                      sort_keys=False,
                      indent=2,
                      ensure_ascii=False)
