"""Reproducible, selective source import. Run once when reviewing upstream gait.

Writes only through apply_patch. No runtime dependency on the old repository.
"""

import ast
import difflib
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT.parent / "Roki_2_Soccer"


def add(path, content):
    target = ROOT / path
    if target.exists():
        old = target.read_text()
        if old == content:
            return
        delta = list(difflib.unified_diff(old.splitlines(True), content.splitlines(True)))
        patch = f"*** Begin Patch\n*** Update File: {target}\n" + "".join("@@\n" if line.startswith("@@") else line for line in delta[2:])
    else:
        patch = f"*** Begin Patch\n*** Add File: {target}\n"
        patch += "".join("+" + line + "\n" for line in content.splitlines())
    subprocess.run(["apply_patch", patch + "*** End Patch\n"], check=True, capture_output=True)


class HardwareOnly(ast.NodeTransformer):
    constants = {"self.glob.SIMULATION": 5, "self.glob.with_Local": False,
                 "self.glob.monitor_is_on": False, "self.glob.record_motions": False,
                 "self.with_Vision": False}

    def visit_FunctionDef(self, node):
        node = self.generic_visit(node)
        if node.name == "walk_Final_Pose":
            # The upstream 233 mm target exceeds our 221.8 mm leg model.
            for child in ast.walk(node):
                if isinstance(child, ast.Constant) and child.value == 233.0:
                    child.value = 215.0
        if node.name == "kick":
            for child in ast.walk(node):
                if isinstance(child, ast.Constant) and child.value == "SOLE_LANDING_SKEW":
                    child.value = "KICK_SOLE_LANDING_SKEW"
        return node

    def visit_Attribute(self, node):
        key = ast.unparse(node)
        return ast.Constant(self.constants[key]) if key in self.constants else self.generic_visit(node)

    def visit_If(self, node):
        node = self.generic_visit(node)
        try:
            # Only evaluate expressions consisting entirely of literal constants.
            if any(isinstance(n, (ast.Name, ast.Attribute, ast.Call)) for n in ast.walk(node.test)):
                return node
            result = eval(compile(ast.fix_missing_locations(ast.Expression(node.test)), "<constant>", "eval"), {"__builtins__": {}})
        except Exception:
            return node
        return node.body if result else node.orelse

    def visit_Assign(self, node):
        if any(ast.unparse(t).startswith("self.local.") for t in node.targets):
            return None
        return self.generic_visit(node)

    def visit_Expr(self, node):
        if (isinstance(node.value, ast.Call) and ast.unparse(node.value.func) == "print"
                and node.value.args and isinstance(node.value.args[0], ast.Constant)
                and node.value.args[0].value == "bad_ik_calc:"):
            return None  # Engine reports IK failures through the worker log channel.
        if isinstance(node.value, ast.Call) and ast.unparse(node.value.func).startswith("self.local."):
            return None
        return self.generic_visit(node)

    def visit_Call(self, node):
        name = ast.unparse(node.func)
        node = self.generic_visit(node)
        if name == "starkit.alpha_calculation":
            leg = "right" if ast.unparse(node.args[0]) == "self.xtr" else "left"
            node.func = ast.Attribute(ast.Name("self", ast.Load()), "solve_leg", ast.Load())
            node.args.insert(0, ast.Constant(leg))
            return node
        if name == "self.Roki.Rcb4.ServoData":
            return ast.Call(ast.Name("Servo", ast.Load()), [], [])
        if name == "self.rcb.setServoPosAsync":
            return ast.Yield(ast.Tuple([ast.Constant("servo"), *node.args], ast.Load()))
        if name == "self.wait_for_gueue_end":
            return ast.Yield(ast.Tuple([ast.Constant("drain")], ast.Load()))
        if name in ("self.walk_Initial_Pose", "self.walk_Final_Pose_After_Kick"):
            if name.endswith("walk_Initial_Pose"):
                node.keywords = [k for k in node.keywords if k.arg != "start_mixing"]
                node.keywords.append(ast.keyword("start_mixing", ast.Constant(False)))
            return ast.YieldFrom(node)
        if name in ("time.sleep", "self.pause_in_ms"):
            value = node.args[0]
            if name.endswith("pause_in_ms"):
                value = ast.BinOp(value, ast.Div(), ast.Constant(1000))
            return ast.Yield(ast.Tuple([ast.Constant("sleep"), value], ast.Load()))
        return node


def main():
    path = SOURCE / "Soccer/Motion/class_Motion.py"
    source = path.read_text()
    tree = ast.parse(source)
    names = {"computeAlphaForWalk", "walk_Initial_Pose", "walk_Cycle", "walk_Final_Pose",
             "walk_Final_Pose_After_Kick", "kick"}
    functions = [n for n in tree.body if isinstance(n, ast.ClassDef)][0].body
    functions = [HardwareOnly().visit(n) for n in functions if isinstance(n, ast.FunctionDef) and n.name in names]
    # Mixing is started once by the hardware adapter; remove obsolete branch entirely.
    initial = next(n for n in functions if n.name == "walk_Initial_Pose")
    initial.body = [n for n in initial.body if not (isinstance(n, ast.If) and "start_mixing" in ast.unparse(n.test))]
    module = ast.Module([ast.ClassDef("GaitAlgorithms", [], [], functions, [])], [])
    ast.fix_missing_locations(module)
    header = ('# Copyright STARKIT Soccer team of MIPT. Adapted from class_Motion.py.\n'
              '# Generated by tools/import_motion.py; hardware output is cooperative yield.\n'
              f'# Source SHA256: {hashlib.sha256(source.encode()).hexdigest()}\n'
              'import math\nimport time\nfrom dataclasses import dataclass\n\n'
              'uprint = print\n\n@dataclass\nclass Servo:\n    Id: int = 0\n    Sio: int = 0\n    Data: int = 0\n\n')
    add("roki_ng/motion/gait.py", header + ast.unparse(module) + "\n")
    robot = (SOURCE / "Robots/class_Robot_Roki_2.py").read_text()
    add("roki_ng/motion/model.py", "\n".join(line.rstrip() for line in robot.splitlines()).rstrip() + "\n")
    real = ast.parse((SOURCE / "Soccer/Motion/class_Motion_real.py").read_text())
    jumps = {}
    for n in ast.walk(real):
        if isinstance(n, ast.Assign) and len(n.targets) == 1:
            target = ast.unparse(n.targets[0])
            if target.startswith("self.jump_motion_"):
                jumps[target.removeprefix("self.jump_motion_")] = ast.literal_eval(n.value)
    add("roki_ng/assets/jumps.json", json.dumps(jumps, indent=2) + "\n")
    for path in sorted((SOURCE / "Soccer/Motion/motion_slots").glob("*.json")):
        data = json.loads(path.read_text())
        if path.stem not in data:
            continue
        add(f"roki_ng/assets/slots/{path.name}", json.dumps(data, separators=(",", ":")) + "\n")


if __name__ == "__main__":
    main()
