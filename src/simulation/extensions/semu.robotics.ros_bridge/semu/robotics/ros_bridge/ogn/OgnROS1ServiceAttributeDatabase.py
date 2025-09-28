"""Support for simplified access to data on nodes of type semu.robotics.ros_bridge.ROS1ServiceAttribute

This node provides the services to list, read and write prim's attributes
"""

import omni.graph.core as og
import carb
import sys
import traceback
class OgnROS1ServiceAttributeDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type semu.robotics.ros_bridge.ROS1ServiceAttribute

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.execIn
            inputs.getAttributeServiceName
            inputs.getAttributesServiceName
            inputs.nodeNamespace
            inputs.primsServiceName
            inputs.setAttributeServiceName
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata, Is_Required, DefaultValue
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:execIn', 'execution', 0, None, 'The input execution port', {}, True, None),
        ('inputs:getAttributeServiceName', 'string', 0, None, "Name of the service to read a specific prim's attribute", {og.MetadataKeys.DEFAULT: '"get_attribute"'}, True, 'get_attribute'),
        ('inputs:getAttributesServiceName', 'string', 0, None, "Name of the service to list all specific prim's attributes", {og.MetadataKeys.DEFAULT: '"get_attributes"'}, True, 'get_attributes'),
        ('inputs:nodeNamespace', 'string', 0, None, 'Namespace of ROS1 Node, prepends any published/subscribed topic, service or action by the node namespace', {og.MetadataKeys.DEFAULT: '""'}, True, ''),
        ('inputs:primsServiceName', 'string', 0, None, 'Name of the service to list all prims in the current stage', {og.MetadataKeys.DEFAULT: '"get_prims"'}, True, 'get_prims'),
        ('inputs:setAttributeServiceName', 'string', 0, None, "Name of the service to write a specific prim's attribute", {og.MetadataKeys.DEFAULT: '"set_attribute"'}, True, 'set_attribute'),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)

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
        def getAttributeServiceName(self):
            data_view = og.AttributeValueHelper(self._attributes.getAttributeServiceName)
            return data_view.get()

        @getAttributeServiceName.setter
        def getAttributeServiceName(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.getAttributeServiceName)
            data_view = og.AttributeValueHelper(self._attributes.getAttributeServiceName)
            data_view.set(value)
            self.getAttributeServiceName_size = data_view.get_array_size()

        @property
        def getAttributesServiceName(self):
            data_view = og.AttributeValueHelper(self._attributes.getAttributesServiceName)
            return data_view.get()

        @getAttributesServiceName.setter
        def getAttributesServiceName(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.getAttributesServiceName)
            data_view = og.AttributeValueHelper(self._attributes.getAttributesServiceName)
            data_view.set(value)
            self.getAttributesServiceName_size = data_view.get_array_size()

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
        def primsServiceName(self):
            data_view = og.AttributeValueHelper(self._attributes.primsServiceName)
            return data_view.get()

        @primsServiceName.setter
        def primsServiceName(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.primsServiceName)
            data_view = og.AttributeValueHelper(self._attributes.primsServiceName)
            data_view.set(value)
            self.primsServiceName_size = data_view.get_array_size()

        @property
        def setAttributeServiceName(self):
            data_view = og.AttributeValueHelper(self._attributes.setAttributeServiceName)
            return data_view.get()

        @setAttributeServiceName.setter
        def setAttributeServiceName(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.setAttributeServiceName)
            data_view = og.AttributeValueHelper(self._attributes.setAttributeServiceName)
            data_view.set(value)
            self.setAttributeServiceName_size = data_view.get_array_size()
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
        self.inputs = OgnROS1ServiceAttributeDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnROS1ServiceAttributeDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnROS1ServiceAttributeDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
    class abi:
        """Class defining the ABI interface for the node type"""
        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnROS1ServiceAttributeDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):
                return get_node_type_function()
            return 'semu.robotics.ros_bridge.ROS1ServiceAttribute'
        @staticmethod
        def compute(context, node):
            db = OgnROS1ServiceAttributeDatabase(node)
            try:
                db.inputs._setting_locked = True
                compute_function = getattr(OgnROS1ServiceAttributeDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:
                    return compute_function(context, node)
                return OgnROS1ServiceAttributeDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
            return False
        @staticmethod
        def initialize(context, node):
            OgnROS1ServiceAttributeDatabase._initialize_per_node_data(node)

            # Set any default values the attributes have specified
            db = OgnROS1ServiceAttributeDatabase(node)
            db.inputs.getAttributeServiceName = "get_attribute"
            db.inputs.getAttributesServiceName = "get_attributes"
            db.inputs.nodeNamespace = ""
            db.inputs.primsServiceName = "get_prims"
            db.inputs.setAttributeServiceName = "set_attribute"
            initialize_function = getattr(OgnROS1ServiceAttributeDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):
                initialize_function(context, node)
        @staticmethod
        def release(node):
            release_function = getattr(OgnROS1ServiceAttributeDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):
                release_function(node)
            OgnROS1ServiceAttributeDatabase._release_per_node_data(node)
        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnROS1ServiceAttributeDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):
                return update_node_version_function(context, node, old_version, new_version)
            return False
        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnROS1ServiceAttributeDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(og.MetadataKeys.EXTENSION, "semu.robotics.ros_bridge")
                node_type.set_metadata(og.MetadataKeys.UI_NAME, "ROS1 Attribute Service")
                node_type.set_metadata(og.MetadataKeys.CATEGORIES, "semuRos:service")
                node_type.set_metadata(og.MetadataKeys.DESCRIPTION, "This node provides the services to list, read and write prim's attributes")
                node_type.set_metadata(og.MetadataKeys.LANGUAGE, "Python")
                icon_path = carb.tokens.get_tokens_interface().resolve("${semu.robotics.ros_bridge}")
                icon_path = icon_path + '/' + "semu/robotics/ros_bridge/ogn/nodes/icons/icon.svg"
                node_type.set_metadata(og.MetadataKeys.ICON_PATH, icon_path)
                OgnROS1ServiceAttributeDatabase.INTERFACE.add_to_node_type(node_type)
        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnROS1ServiceAttributeDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):
                on_connection_type_resolve_function(node)
    NODE_TYPE_CLASS = None
    GENERATOR_VERSION = (1, 3, 5)
    TARGET_VERSION = (2, 27, 0)
    @staticmethod
    def register(node_type_class):
        OgnROS1ServiceAttributeDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnROS1ServiceAttributeDatabase.abi, 1)
    @staticmethod
    def deregister():
        og.deregister_node_type("semu.robotics.ros_bridge.ROS1ServiceAttribute")
