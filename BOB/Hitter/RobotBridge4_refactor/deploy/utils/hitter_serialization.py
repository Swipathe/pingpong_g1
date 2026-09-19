from __future__ import annotations

import math
from types import MappingProxyType
from typing import Any, Dict, Mapping, Tuple, Union

import numpy as np

JsonScalar = Union[None, bool, int, float, str]
JsonValue = Union[
    JsonScalar,
    Tuple["JsonValue", ...],
    Mapping[str, "JsonValue"],
]


def freeze_json_value(value: Any) -> JsonValue:
    """Return a detached, recursively immutable JSON-compatible value."""
    if isinstance(value, float):
        if math.isnan(value):
            return "NaN"
        if math.isinf(value):
            return "Infinity" if value > 0.0 else "-Infinity"
        return value
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, np.generic):
        return freeze_json_value(value.item())
    if isinstance(value, np.ndarray):
        return freeze_json_value(value.tolist())
    if isinstance(value, Mapping):
        frozen: Dict[str, JsonValue] = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise TypeError("JSON mapping keys must be strings")
            frozen[key] = freeze_json_value(item)
        return MappingProxyType(frozen)
    if isinstance(value, (tuple, list)):
        return tuple(freeze_json_value(item) for item in value)
    raise TypeError("value is not JSON-compatible: {!r}".format(type(value).__name__))


def to_builtin_json(value: Any) -> Any:
    """Convert a frozen JSON value to fresh dict/list/scalar builtins."""
    if isinstance(value, float):
        if math.isnan(value):
            return "NaN"
        if math.isinf(value):
            return "Infinity" if value > 0.0 else "-Infinity"
        return value
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, np.generic):
        return to_builtin_json(value.item())
    if isinstance(value, np.ndarray):
        return [to_builtin_json(item) for item in value.tolist()]
    if isinstance(value, Mapping):
        return {str(key): to_builtin_json(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [to_builtin_json(item) for item in value]
    raise TypeError("value is not JSON-compatible: {!r}".format(type(value).__name__))
