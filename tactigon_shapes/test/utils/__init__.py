from os import path

from tactigon_shapes.models import BASE_PATH

def read_resource(file_path: str):
    with open(get_resource_path(file_path)) as content:
        return content
    
def get_resource_path(file_path: str) -> str:
    return path.join(BASE_PATH, "tactigon_shapes", "test", "resources", file_path)