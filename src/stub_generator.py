from __future__ import annotations

import re
from pathlib import Path

import yaml

from .code_generator import CodeGenerator

MODULE_NAMES = ["carla", "command"]


class CarlaStubGenerator:
    module_name: str
    classes: list[dict]
    _generator: CodeGenerator

    def __init__(self, file):
        try:
            data = yaml.safe_load(file)[0]
        except IndexError:
            raise IndexError(f"{file} is empty")

        self.module_name = data.get("module_name")
        self.classes = data.get("classes")
        self._generator = CodeGenerator()
        self._generator.add_line("from __future__ import annotations")
        self._generator.add_line()
        self._generator.add_line("from typing import Any")
        self._generator.add_line("import numpy as np")
        self._generator.add_line()
        for module_name in MODULE_NAMES:
            if self.module_name != module_name:
                self._generator.add_line(f"import {module_name}")
        self._generator.add_line()

    def save_to_file(self, directory: Path):
        self._generate()
        file_name = "__init__" if self.module_name == "carla" else self.module_name
        self._generator.save_to_file(directory / f"{file_name}.pyi")

    def _generate(self):
        for _class in self.classes:
            self._generate_class(_class)
            self._generator.add_line()
            self._generator.add_line()

    def _generate_class(self, _class):
        class_name = _class.get("class_name", None)
        if class_name:
            parent = _class.get("parent", None)
            if parent:
                parent = self._remove_module(parent)
                self._generator.add_line(f"class {class_name}({parent}):")
            else:
                self._generator.add_line(f"class {class_name}:")
            self._generator.indent()
            self._generator.add_line('"""')
            self._generator.add_line(self._parse_docs(_class.get("doc", "")))
            self._generator.add_line('"""')
            self._generator.add_line()
            self._generate_instance_variables(_class)
            self._generate_methods(_class)
            self._generator.dedent()

    def _generate_instance_variables(self, _class):
        instance_variables = _class.get("instance_variables", [])
        if instance_variables:
            for instance_variable in instance_variables:
                name = instance_variable.get("var_name", None)
                if name:
                    type = self._parse_type(instance_variable.get("type", "Any"))
                    doc = self._parse_docs(instance_variable.get("doc", ""))
                    self._generator.add_line(f"{name}: {type}")
                    self._generator.add_line(f'"""{doc}"""')
                    self._generator.add_line()

    def _generate_methods(self, _class):
        methods = _class.get("methods", [])
        for method in methods:
            name = method.get("def_name", None)
            if name:
                param_list = method.get("params", [])
                param_list = param_list or []

                # parse params in method defintion
                def_params = "self, "
                for param in param_list:
                    param_name = param.get("param_name", "").split("(")[0]
                    param_type = self._parse_type(param.get("type", "Any"))
                    param_default = param.get("default", None)
                    if param_default is not None:
                        if param_type == "str":
                            param_default = f'"{param_default}"'
                        try:
                            param_default = self._remove_module(param_default)
                            if param_default.startswith(f"{param_type}."):
                                param_default = param_default.split(".")[0]
                        except:
                            pass
                        def_params += f"{param_name}: {param_type} = {param_default}, "
                    else:
                        def_params += f"{param_name}: {param_type}, "
                def_params = def_params.rstrip(", ")

                # parse return type and generate method definition
                return_type = method.get("return", None)
                if return_type:
                    parsed_return_type = self._parse_type(return_type)
                    self._generator.add_line(
                        f"def {name}({def_params}) -> {parsed_return_type}:"
                    )
                else:
                    self._generator.add_line(f"def {name}({def_params}):")

                # generate method docstring
                self._generator.indent()
                self._generator.add_line('"""')
                self._generator.add_line(self._parse_docs(method.get("doc", "")))

                note = method.get("note", None)
                if note:
                    self._generator.add_line()
                    self._generator.add_line(f"*note*: {self._parse_docs(note)}")

                warning = method.get("warning", None)
                if warning:
                    self._generator.add_line()
                    self._generator.add_line(
                        f"**warning**: {self._parse_docs(warning)}"
                    )

                for param in param_list:
                    param_name = param.get("param_name", "")
                    param_type = self._parse_type(param.get("type", ""))
                    param_doc = self._parse_docs(param.get("doc", ""))
                    self._generator.add_line()
                    self._generator.add_line(
                        f":param {param_name}: ({param_type}) {param_doc}"
                    )

                if return_type:
                    self._generator.add_line()
                    self._generator.add_line(f":return: {return_type}")

                self._generator.add_line('"""')
                self._generator.add_line("...")
                self._generator.dedent()
                self._generator.add_line()

    def _parse_type(self, _type: str):
        if "uint" in _type:
            return f"np.{_type}"
        if _type == "boolean":
            return "bool"
        if "<" in _type:  # skip html tags
            return "Any"
        if " " in _type:
            return "str"
        if _type == "string":
            return "str"
        _type = self._remove_module(_type)  # remove module name
        _type = _type.replace("(", "[").replace(")", "]")

        return _type

    def _remove_module(self, value: str):
        return value.replace(f"{self.module_name}.", "")

    def _parse_docs(self, value: str):
        # remove html tags of type __<font color="#7fb800">add_force()</font>__
        value = re.sub(r"__<.*?>", "", value)
        value = re.sub(r"<.*?>__", "", value)

        # remove other tags
        for tag in ["a", "b", "i", "br", "code", "font", "small"]:
            value = re.sub(rf"<{tag}.*?>", "", value)
            value = value.replace(f"</{tag}>", "")

        return value.strip()
