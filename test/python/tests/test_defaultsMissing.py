import unittest
from rosinterface_handler.param.DefaultsMissingInterface import DefaultsMissingInterface


class TestDefaultsMissingInterface(unittest.TestCase):
    def test_defaults_missing(self):
        with self.assertRaises(KeyError):
            params = DefaultsMissingInterface()
