import omni.kit.test
import omni.graph.core as og
import omni.graph.core.tests as ogts
import os
import carb


class TestOgn(ogts.test_case_class(use_schema_prims=True, allow_implicit_graph=False)):

    async def test_import(self):
        import semu.robotics.ros_bridge.ogn.OgnROS1ServiceAttributeDatabase
        self.assertTrue(hasattr(semu.robotics.ros_bridge.ogn.OgnROS1ServiceAttributeDatabase, "OgnROS1ServiceAttributeDatabase"))

    async def test_usda(self):
        test_file_name = "OgnROS1ServiceAttributeTemplate.usda"
        usd_path = os.path.join(os.path.dirname(__file__), "usd", test_file_name)
        if not os.path.exists(usd_path):
            self.assertTrue(False, f"{usd_path} not found for loading test")
        (result, error) = await ogts.load_test_file(usd_path)
        self.assertTrue(result, f'{error} on {usd_path}')
        test_node = og.Controller.node("/TestGraph/Template_semu_robotics_ros_bridge_ROS1ServiceAttribute")
        self.assertTrue(test_node.is_valid())
        node_type_name = test_node.get_type_name()
        self.assertEqual(og.GraphRegistry().get_node_type_version(node_type_name), 1)
        self.assertTrue(test_node.get_attribute_exists("inputs:execIn"))

        self.assertTrue(test_node.get_attribute_exists("inputs:getAttributeServiceName"))

        input_attr = test_node.get_attribute("inputs:getAttributeServiceName")
        actual_input = og.Controller.get(input_attr)
        ogts.verify_values("get_attribute", actual_input, "semu.robotics.ros_bridge.ROS1ServiceAttribute USD load test - inputs:getAttributeServiceName attribute value error")
        self.assertTrue(test_node.get_attribute_exists("inputs:getAttributesServiceName"))

        input_attr = test_node.get_attribute("inputs:getAttributesServiceName")
        actual_input = og.Controller.get(input_attr)
        ogts.verify_values("get_attributes", actual_input, "semu.robotics.ros_bridge.ROS1ServiceAttribute USD load test - inputs:getAttributesServiceName attribute value error")
        self.assertTrue(test_node.get_attribute_exists("inputs:nodeNamespace"))

        input_attr = test_node.get_attribute("inputs:nodeNamespace")
        actual_input = og.Controller.get(input_attr)
        ogts.verify_values("", actual_input, "semu.robotics.ros_bridge.ROS1ServiceAttribute USD load test - inputs:nodeNamespace attribute value error")
        self.assertTrue(test_node.get_attribute_exists("inputs:primsServiceName"))

        input_attr = test_node.get_attribute("inputs:primsServiceName")
        actual_input = og.Controller.get(input_attr)
        ogts.verify_values("get_prims", actual_input, "semu.robotics.ros_bridge.ROS1ServiceAttribute USD load test - inputs:primsServiceName attribute value error")
        self.assertTrue(test_node.get_attribute_exists("inputs:setAttributeServiceName"))

        input_attr = test_node.get_attribute("inputs:setAttributeServiceName")
        actual_input = og.Controller.get(input_attr)
        ogts.verify_values("set_attribute", actual_input, "semu.robotics.ros_bridge.ROS1ServiceAttribute USD load test - inputs:setAttributeServiceName attribute value error")
