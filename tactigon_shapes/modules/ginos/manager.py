from .models import LLMMessageRole

def get_ginos_blocks():
    return dict(
        roles=[(role.name, role.value) for role in LLMMessageRole]
    )