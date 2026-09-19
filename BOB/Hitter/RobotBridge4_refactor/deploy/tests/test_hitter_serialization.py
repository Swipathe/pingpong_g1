from __future__ import annotations

from types import MappingProxyType

import numpy as np
import pytest


def test_freeze_json_value_detaches_and_recursively_freezes_nested_values() -> None:
    from utils.hitter_serialization import freeze_json_value

    source = {
        "metrics": [1, {"state": "ready"}],
        "samples": np.array([float("nan"), float("inf"), float("-inf")]),
    }

    frozen = freeze_json_value(source)
    source["metrics"][1]["state"] = "mutated"
    source["metrics"].append(2)
    source["samples"][0] = 0.0

    assert isinstance(frozen, MappingProxyType)
    assert frozen["metrics"] == (1, MappingProxyType({"state": "ready"}))
    assert frozen["samples"] == ("NaN", "Infinity", "-Infinity")
    with pytest.raises(TypeError):
        frozen["other"] = 1
    with pytest.raises(TypeError):
        frozen["metrics"][1]["state"] = "changed"


def test_to_builtin_json_returns_fresh_nested_containers() -> None:
    from utils.hitter_serialization import freeze_json_value, to_builtin_json

    frozen = freeze_json_value({"items": [{"value": 1}]})

    first = to_builtin_json(frozen)
    second = to_builtin_json(frozen)
    first["items"][0]["value"] = 99

    assert first == {"items": [{"value": 99}]}
    assert second == {"items": [{"value": 1}]}
    assert to_builtin_json(frozen) == {"items": [{"value": 1}]}
    assert first is not second
    assert first["items"] is not second["items"]
