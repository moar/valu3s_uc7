"""Support for simplified access to data on nodes of type semu.robotics.ros_bridge.ROS1ActionGripperCommand

This node provides the GripperCommand action server to control a robotic gripper
"""

import omni.graph.core as og
import carb
import sys
import traceback
class OgnROS1ActionGripperCommandDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type semu.robotics.ros_bridge.ROS1ActionGripperCommand

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.actionNamespace
            inputs.controllerName
            inputs.execIn
            inputs.nodeNamespace
            inputs.targetGripperJoints
            inputs.targetPrim
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata, Is_Required, DefaultValue
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:actionNamespace', 'string', 0, None, 'Action namespace for the controller', {og.MetadataKeys.DEFAULT: '"gripper_command"'}, True, 'gripper_command'),
        ('inputs:controllerName', 'string', 0, None, 'Name of the controller', {og.MetadataKeys.DEFAULT: '"gripper_controller"'}, True, 'gripper_controller'),
        ('inputs:execIn', 'execution', 0, None, 'The input execution port', {}, True, None),
        ('inputs:nodeNamespace', 'string', 0, None, 'Namespace of ROS1 Node, prepends any published/subscribed topic, service or action by the node namespace', {og.MetadataKeys.DEFAULT: '""'}, True, ''),
        ('inputs:targetGripperJoints', 'bundle', 0, None, 'USD reference to the gripper joints', {}, True, None),
        ('inputs:targetPrim', 'bundle', 0, None, 'USD reference to the robot prim', {}, True, None),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.Database.ROLE_EXECUTION
        role_data.inputs.targetGripperJoints = og.Database.ROLE_BUNDLE
        role_data.inputs.targetPrim = og.Database.ROLE_BUNDLE
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.__bundles = og.BundleContainer(context, node, attributes, [], read_only=True)

        @property
        def actionNamespace(self):
            data_view = og.AttributeValueHelper(self._attributes.actionNamespace)
            return data_view.get()

        @actionNamespace.setter
        def actionNamespace(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.actionNamespace)
            data_view = og.AttributeValueHelper(self._attributes.actionNamespace)
            data_view.set(value)
            self.actionNamespace_size = data_view.get_array_size()

        @property
        def controllerName(self):
            data_view = og.AttributeValueHelper(self._attributes.controllerName)
            return data_view.get()

        @controllerName.setter
        def controllerName(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.controllerName)
            data_view = og.AttributeValueHelper(self._attributes.controllerName)
            data_view.set(value)
            self.controllerName_size = data_view.get_array_size()

        @property
        def execIn(self):
            data_view = og.AttributeValueHelper(self._attributes.execIn)
            return data_view.get()

        @execIn.setter
        def execIn(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.execIn)
            data_view = og.AttributeValueHelper(self._attributes.execIn)
            data_view.set(value)

        @property
        def nodeNamespace(self):
            data_view = og.AttributeValueHelper(self._attributes.nodeNamespace)
            return data_view.get()

        @nodeNamespace.setter
        def nodeNamespace(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.nodeNamespace)
            data_view = og.AttributeValueHelper(self._attributes.nodeNamespace)
            data_view.set(value)
            self.nodeNamespace_size = data_view.get_array_size()

        @property
        def targetGripperJoints(self) -> og.BundleContents:
            """Get the bundle wrapper class for the attribute inputs.targetGripperJoints"""
            return self.__bundles.targetGripperJoints

        @property
        def targetPrim(self) -> og.BundleContents:
            """Get the bundle wrapper class for the attribute inputs.targetPrim"""
            return self.__bundles.targetPrim
    class ValuesForOutputs(og.DynamicAttributeAccess):
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
    class ValuesForState(og.DynamicAttributeAccess):
        """Helper class that creates natural hierarchical access to state attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
    def __init__(self, node):
        super().__init__(node)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT)
        self.inputs = OgnROS1ActionGripperCommandDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnROS1ActionGripperCommandDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnROS1ActionGripperCommandDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
    class abi:
        """Class defining the ABI interface for the node type"""
        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnROS1ActionGripperCommandDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):
                return get_node_type_function()
            return 'semu.robotics.ros_bridge.ROS1ActionGripperCommand'
        @staticmethod
        def compute(context, node):
            db = OgnROS1ActionGripperCommandDatabase(node)
            try:
                db.inputs._setting_locked = True
                compute_function = getattr(OgnROS1ActionGripperCommandDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:
                    return compute_function(context, node)
                return OgnROS1ActionGripperCommandDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
            return False
        @staticmethod
        def initialize(context, node):
            OgnROS1ActionGripperCommandDatabase._initialize_per_node_data(node)

            # Set any default values the attributes have specified
            db = OgnROS1ActionGripperCommandDatabase(node)
            db.inputs.actionNamespace = "gripper_command"
            db.inputs.controllerName = "gripper_controller"
            db.inputs.nodeNamespace = ""
            initialize_function = getattr(OgnROS1ActionGripperCommandDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):
                initialize_function(context, node)
        @staticmethod
        def release(node):
            release_function = getattr(OgnROS1ActionGripperCommandDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):
                release_function(node)
            OgnROS1ActionGripperCommandDatabase._release_per_node_data(node)
        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnROS1ActionGripperCommandDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):
                return update_node_version_function(context, node, old_version, new_version)
            return False
        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnROS1ActionGripperCommandDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(og.MetadataKeys.EXTENSION, "semu.robotics.ros_bridge")
                node_type.set_metadata(og.MetadataKeys.UI_NAME, "ROS1 GripperCommand Action")
                node_type.set_metadata(og.MetadataKeys.CATEGORIES, "semuRos:action")
                node_type.set_metadata(og.MetadataKeys.DESCRIPTION, "This node provides the GripperCommand action server to control a robotic gripper")
                node_type.set_metadata(og.MetadataKeys.LANGUAGE, "Python")
                icon_path = carb.tokens.get_tokens_interface().resolve("${semu.robotics.ros_bridge}")
                icon_path = icon_path + '/' + "semu/robotics/ros_bridge/ogn/nodes/icons/icon.svg"
                node_type.set_metadata(og.MetadataKeys.ICON_PATH, icon_path)
                OgnROS1ActionGripperCommandDatabase.INTERFACE.add_to_node_type(node_type)
        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnROS1ActionGripperCommandDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):
                on_connection_type_resolve_function(node)
    NODE_TYPE_CLASS = None
    GENERATOR_VERSION = (1, 3, 5)
    TARGET_VERSION = (2, 27, 0)
    @staticmethod
    def register(node_type_class):
        OgnROS1ActionGripperCommandDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnROS1ActionGripperCommandDatabase.abi, 1)
    @staticmethod
    def deregister():
        og.deregister_node_type("semu.robotics.ros_bridge.ROS1ActionGripperCommand")
