import unittest
from rosinterface_handler.interface.DefaultsAtLaunchInterface import DefaultsAtLaunchInterface


class TestDefaultsAtLaunch(unittest.TestCase):
    def test_defaults_at_launch(self):
        params = DefaultsAtLaunchInterface()
        self.assertEqual(params.int_param_wo_default, 1)
        self.assertAlmostEqual(params.double_param_wo_default, 1.1)
        self.assertEqual(params.str_param_wo_default, "Hello World")
        self.assertEqual(params.bool_param_wo_default, True)

        self.assertEqual(params.vector_int_param_wo_default, [1, 2, 3])
        self.assertEqual(params.vector_double_param_wo_default, [1.1, 1.2, 1.3])
        self.assertEqual(params.vector_string_param_wo_default, ["Hello", "World"])

        self.assertEqual(params.map_param_wo_default, {"Hello": "World"})
        self.assertEqual(params.enum_param_wo_default, 1)

    def test_defaults_at_launch_subscriber(self):
        params = DefaultsAtLaunchInterface()
        self.assertEqual(params.subscriber_wo_default.sub.name, "/test/rosinterface_handler_python_test/in_topic")

    def test_defaults_at_launch_publisher(self):
        params = DefaultsAtLaunchInterface()
        self.assertEqual(params.publisher_wo_default.name, "/test/rosinterface_handler_python_test/out_topic")
