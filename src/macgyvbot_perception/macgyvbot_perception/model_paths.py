"""Shared model path resolution helpers."""

from macgyvbot_resources.resources import resolve_weight_file


def resolve_weight_path(model_name, default_model_name=None):
    """Resolve a model file from macgyvbot_resources."""
    return resolve_weight_file(model_name, default_model_name)
