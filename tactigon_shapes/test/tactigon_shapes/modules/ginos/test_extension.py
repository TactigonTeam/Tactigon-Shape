from unittest import TestCase

import pandas as pd

from tactigon_shapes.modules.ollama.extension import OllamaInterface
from tactigon_shapes.modules.ollama.models import OllamaConfig

from tactigon_shapes.test.utils import get_resource_path

class TestOllamaExtension(TestCase):
    def setUp(self):
        self._config = OllamaConfig(
            "http://127.0.0.1:11434/",
            "tinyllama"
        )
        self._ginos = OllamaInterface(**self._config.toJSON())