import bpy
from bpy_extras.io_utils import ImportHelper
from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty, BoolProperty, EnumProperty
from bpy.types import Operator

from.import ndMeshImport
from.import ndMeshExport

# blend installation info
bl_info = {
    "name": "Newton Dynamics file format",
    "author": "Julio Jerez",
    "version": (1, 0, 0),
    "blender": (5, 0, 0),
    "location": "File > Import-Export",
    "description": "Import-Export nd",
    "warning": "",
    "wiki_url": "http://newtondynamics.com",
    "support": 'COMMUNITY',
    "category": "Import-Export"}

# import mesh class  hook
class ImportMesh(Operator, ImportHelper):
    """This appears in the tooltip of the operator and in the generated docs"""
    bl_idname = "import_test.some_data"  # important since its how bpy.ops.import_test.some_data is constructed
    bl_label = "import ndMesh"

    # ImportHelper mix-in class uses this.
    filename_ext = ".nd"

    filter_glob: StringProperty(
        default="*.nd",
        options={'HIDDEN'},
        maxlen=255,  # Max internal buffer length, longer would be clamped.
    )  # type: ignore

    # List of operator properties, the attributes will be assigned
    # to the class instance from the operator settings before calling.
    use_setting: BoolProperty(
        name="Example Boolean",
        description="Example Tooltip",
        default=True,
    )  # type: ignore

    type: EnumProperty(
        name="Example Enum",
        description="Choose between two items",
        items=(
            ('OPT_A', "First Option", "Description one"),
            ('OPT_B', "Second Option", "Description two"),
        ),
        default='OPT_A',
    ) # type: ignore

    def execute(self, context):
        return ndMeshImport.LoadMesh(context, self.filepath, self.use_setting)

class ExportMesh(Operator, ExportHelper):
    """This appears in the tooltip of the operator and in the generated docs"""
    bl_idname = "export_test.some_data"  # Important since its how bpy.ops.import_test.some_data is constructed.
    bl_label = "export ndMesh"

    # ExportHelper mix-in class uses this.
    filename_ext = ".nd"

    filter_glob: StringProperty(
        default="*.nd",
        options={'HIDDEN'},
        maxlen=255,  # Max internal buffer length, longer would be clamped.
    ) # type: ignore

    # List of operator properties, the attributes will be assigned
    # to the class instance from the operator settings before calling.
    use_setting: BoolProperty(
        name="Example Boolean",
        description="Example Tooltip",
        default=True,
    ) # type: ignore

    type: EnumProperty(
        name="Example Enum",
        description="Choose between two items",
        items=(
            ('OPT_A', "First Option", "Description one"),
            ('OPT_B', "Second Option", "Description two"),
        ),
        default='OPT_A',
    ) # type: ignore

    def execute(self, context):
        object = context.object
        if (object != None):
            ndMeshExport.SaveMesh(context, self.filepath)
        return {'FINISHED'}

# import a mesh browser 
def menu_func_import(self, context):
    self.layout.operator(ImportMesh.bl_idname, text="ndMesh (.nd)")

# export a mesh browser
def menu_func_export(self, context):
    self.layout.operator(ExportMesh.bl_idname, text="ndMesh (.nd)")

# register classes.
def register():
    bpy.utils.register_class(ExportMesh)
    bpy.utils.register_class(ImportMesh)
    
    bpy.types.TOPBAR_MT_file_export.append(menu_func_export)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)

def unregister():
    bpy.utils.unregister_class(ImportMesh)
    bpy.utils.unregister_class(ExportMesh)
    
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_export)

if __name__ == "__main__":
    register()
