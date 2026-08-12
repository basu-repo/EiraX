from __future__ import annotations

from typing import Any, Callable, Optional, TypeVar

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.parameter import Parameter

T = TypeVar("T")
_DYNAMIC_DESCRIPTOR = ParameterDescriptor(dynamic_typing=True)


def declare_yaml_param(
    node: Node,
    name: str,
    *,
    descriptor: Optional[ParameterDescriptor] = None,
) -> None:
    node.declare_parameter(name, descriptor=descriptor or _DYNAMIC_DESCRIPTOR)


def required_param_value(
    node: Node,
    name: str,
    *,
    owner: Optional[str] = None,
) -> Any:
    param = node.get_parameter(name)
    if param.type_ == Parameter.Type.NOT_SET or param.value is None:
        node_name = owner or node.get_name()
        raise ValueError(
            f"{node_name} requires parameter {name!r}; set it in run_follow_defaults.yaml "
            "or provide a launch override"
        )
    return param.value


def yaml_param(
    node: Node,
    name: str,
    *,
    descriptor: Optional[ParameterDescriptor] = None,
    cast: Optional[Callable[[Any], T]] = None,
    owner: Optional[str] = None,
) -> T | Any:
    declare_yaml_param(node, name, descriptor=descriptor)
    value = required_param_value(node, name, owner=owner)
    return cast(value) if cast is not None else value
