import bpy


def read_some_data(context, filepath, use_some_setting):
    print(filepath)
    print("running read_some_data...")
    f = open(filepath, "r", encoding="utf-8")
    data = f.read()
    f.close()

    # Would normally load the data here.
    print(data)

    return {'FINISHED'}