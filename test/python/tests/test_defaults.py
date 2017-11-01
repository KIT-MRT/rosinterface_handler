import unittest
from rosinterface_handler.interface.DefaultsInterface import DefaultsInterface
import rospy


class TestDefaults(unittest.TestCase):
    def test_defaults(self):
        """
        tests defaults on server
        :return:
        """
        params = DefaultsInterface()
        # make sure from_param_server can be called repeatedly
        params.from_param_server()

        self.assertEqual(params.verbosity_param_w_default, 'info')

        self.assertEqual(params.int_param_w_default, 1)
        self.assertAlmostEqual(params.double_param_w_default, 1.1)
        self.assertEqual(params.str_param_w_default, "Hello World")
        self.assertEqual(params.bool_param_w_default, True)

        self.assertEqual(params.vector_int_param_w_default, [1, 2, 3])
        self.assertEqual(params.vector_double_param_w_default, [1.1, 1.2, 1.3])
        self.assertEqual(params.vector_string_param_w_default, ["Hello", "World"])

        self.assertEqual(params.map_param_w_default, {"Hello": "World"})
        self.assertEqual(params.enum_int_param_w_default, 1)
        self.assertEqual(params.enum_str_param_w_default, "One")

    def test_defaults_subscriber(self):
        params = DefaultsInterface()
        self.assertEqual(params.subscriber_w_default.sub.name, "/test/rosinterface_handler_python_test/in_topic")

    def test_defaults_publisher(self):
        params = DefaultsInterface()
        self.assertEqual(params.publisher_w_default.name, "/test/rosinterface_handler_python_test/out_topic")

    def test_defaults_on_server(self):
        params = DefaultsInterface()
        # now all parameters should be set on param server
        self.assertEqual(params.verbosity_param_w_default, rospy.get_param("~verbosity_param_w_default"))

        self.assertEqual(params.int_param_w_default, rospy.get_param("~int_param_w_default"))
        self.assertAlmostEqual(params.double_param_w_default, rospy.get_param("~double_param_w_default"))
        self.assertEqual(params.str_param_w_default, rospy.get_param("~str_param_w_default"))
        self.assertEqual(params.bool_param_w_default, rospy.get_param("~bool_param_w_default"))

        self.assertEqual(params.vector_int_param_w_default, rospy.get_param("~vector_int_param_w_default"))
        self.assertEqual(params.vector_double_param_w_default, rospy.get_param("~vector_double_param_w_default"))
        self.assertEqual(params.vector_string_param_w_default, rospy.get_param("~vector_string_param_w_default"))

        self.assertEqual(params.map_param_w_default, rospy.get_param("~map_param_w_default"))
        self.assertEqual(params.enum_int_param_w_default, rospy.get_param("~enum_int_param_w_default"))
        self.assertEqual(params.enum_str_param_w_default, rospy.get_param("~enum_str_param_w_default"))

    def test_set_parameters_on_server(self):
        params = DefaultsInterface()

        params.verbosity_param_w_default = 'warning'
        params.int_param_w_default = 2
        params.double_param_w_default = 2.2
        params.str_param_w_default = "World Hello"
        params.bool_param_w_default = False
        params.vector_int_param_w_default = [3, 2, 1]
        params.vector_double_param_w_default = [1.3, 1.2, 1.2]
        params.vector_bool_param_w_default = [True, False]
        params.vector_string_param_w_default = ["World", "Hello"]
        params.map_param_w_default = {"World": "Hello"}
        params.enum_int_param_w_default = 2
        params.enum_str_param_w_default = "Two"

        # Upload parameters
        params.to_param_server()

        # now all parameters should be set on param server
        self.assertEqual(params.verbosity_param_w_default, rospy.get_param("~verbosity_param_w_default"))

        self.assertEqual(params.int_param_w_default, rospy.get_param("~int_param_w_default"))
        self.assertAlmostEqual(params.double_param_w_default, rospy.get_param("~double_param_w_default"))
        self.assertEqual(params.str_param_w_default, rospy.get_param("~str_param_w_default"))
        self.assertEqual(params.bool_param_w_default, rospy.get_param("~bool_param_w_default"))

        self.assertEqual(params.vector_int_param_w_default, rospy.get_param("~vector_int_param_w_default"))
        self.assertEqual(params.vector_double_param_w_default, rospy.get_param("~vector_double_param_w_default"))
        self.assertEqual(params.vector_string_param_w_default, rospy.get_param("~vector_string_param_w_default"))

        self.assertEqual(params.map_param_w_default, rospy.get_param("~map_param_w_default"))
        self.assertEqual(params.enum_int_param_w_default, rospy.get_param("~enum_int_param_w_default"))
        self.assertEqual(params.enum_str_param_w_default, rospy.get_param("~enum_str_param_w_default"))