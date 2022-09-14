from __future__ import annotations

class CodeGenerator:
    level: int
    code: str

    def __init__(self):
        self.level = 0
        self.code = ""

    def indent(self):
        self.level += 1

    def dedent(self):
        self.level = max(0, self.level - 1)

    def add_line(self, value: str | None = None):
        if not value:
            self.code += "\n"
        else:
            self.code += "\t" * self.level + str(value) + "\n"

    def save_to_file(self, path: str):
        with open(path, "a") as f:
            f.write(self.code)
