from unittest import TestCase

import pandas as pd

from tactigon_shapes.modules.ginos.extension import GinosInterface
from tactigon_shapes.modules.ginos.models import GinosConfig

from tactigon_shapes.test.utils import get_resource_path

class TestGinosExtension(TestCase):
    def setUp(self):
        self._config = GinosConfig(
            "http://127.0.0.1:11434/",
            "tinyllama"
        )
        self._ginos = GinosInterface(**self._config.toJSON())

    def test_valid_file(self):
        df = self._ginos.file_to_dataframe(get_resource_path("example_dataframe.csv"))
        self.assertIsNotNone(df)
        self.assertIsInstance(df, pd.DataFrame)

    def test_invalid_file(self):
        df = self._ginos.file_to_dataframe(get_resource_path("missing-file.csv"))
        self.assertIsNone(df)

    def test_load_dataframe(self):
        actual = self._ginos.add_file_to_context(get_resource_path("example_dataframe.csv"))

        self.assertTrue(actual)
        self.assertEqual(len(self._ginos._dataframe), 5)

    def test_clear_dataframe(self):
        self._ginos.add_file_to_context(get_resource_path("example_dataframe.csv"))
        self._ginos.clear_context()

        self.assertEqual(len(self._ginos._dataframe), 0)