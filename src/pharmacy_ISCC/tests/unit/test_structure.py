"""Static checks; no ROS, cameras, sockets or robot processes are started."""
import ast
import hashlib
import json
import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from pharmacy_iscc.config import settings
from pharmacy_iscc.config.paths import package_path

PACKAGE = Path(__file__).resolve().parents[2]
LIBRARY = PACKAGE / "src/pharmacy_iscc"
BASELINE = PACKAGE.parent / "pharmacy_pkg"


class StructureTests(unittest.TestCase):
    def test_python_syntax_and_internal_imports(self):
        modules = {}
        for path in LIBRARY.rglob("*.py"):
            name = ".".join(path.relative_to(PACKAGE / "src").with_suffix("").parts)
            if name.endswith(".__init__"):
                name = name[:-9]
            modules[name] = ast.parse(path.read_text(encoding="utf-8"), str(path))
        graph = {name: set() for name in modules}
        for name, tree in modules.items():
            for node in ast.walk(tree):
                if isinstance(node, ast.ImportFrom) and (node.module or "").startswith("pharmacy_iscc"):
                    self.assertIn(node.module, modules)
                    graph[name].add(node.module)
                    target = modules[node.module]
                    bound = {n.id for n in ast.walk(target) if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Store)}
                    bound.update(n.name for n in target.body if isinstance(n, (ast.FunctionDef, ast.ClassDef)))
                    bound.update(a.asname or a.name for n in target.body
                                 if isinstance(n, (ast.Import, ast.ImportFrom)) for a in n.names)
                    for alias in node.names:
                        self.assertIn(alias.name, bound, (name, node.module, alias.name))
        def visit(name, ancestors):
            self.assertNotIn(name, ancestors, "Circular package import: " + name)
            for dependency in graph[name]:
                visit(dependency, ancestors + [name])
        for name in graph:
            visit(name, [])
        for path in (PACKAGE / "scripts").glob("*.py"):
            ast.parse(path.read_text(encoding="utf-8"), str(path))
            self.assertLess(len(path.read_text().splitlines()), 12)

    def test_launch_paths_nodes_and_include_arguments(self):
        cmake = (PACKAGE / "CMakeLists.txt").read_text()
        for path in (PACKAGE / "launch").glob("*.launch"):
            tree = ET.parse(path)
            args = {node.get("name") for node in tree.findall("./arg")}
            for node in tree.iter():
                for value in node.attrib.values():
                    for arg in re.findall(r"\$\(arg ([^)]+)\)", value):
                        self.assertIn(arg, args, (path.name, arg))
                if node.tag == "node" and node.get("pkg") == "pharmacy_iscc":
                    self.assertTrue((PACKAGE / "scripts" / node.get("type")).is_file())
                    self.assertIn("scripts/" + node.get("type"), cmake)
                if node.tag == "include" and "$(find pharmacy_iscc)" in node.get("file", ""):
                    target = Path(node.get("file").replace("$(find pharmacy_iscc)", str(PACKAGE)))
                    self.assertTrue(target.is_file())
                    target_args = {n.get("name") for n in ET.parse(target).findall("./arg")}
                    for arg in node.findall("./arg"):
                        self.assertIn(arg.get("name"), target_args)
        ET.parse(PACKAGE / "package.xml")

    def test_resource_paths_and_defaults(self):
        self.assertEqual(settings.COMMON["board2"]["result_source"], "tcp")
        self.assertIsNone(settings.COMMON["board2"]["force_label_for_debug"])
        self.assertEqual(settings.COMMON["board1"]["stable_frames"], 3)
        self.assertFalse(settings.COMMON["board1"]["window_complete_from_three"])
        self.assertTrue(settings.CARS[1]["start_active"])
        self.assertFalse(settings.CARS[2]["start_active"])
        self.assertTrue(settings.CARS[2]["use_peer_board1_result"])
        audio = Path(settings.COMMON["paths"]["audio_dir"])
        for info in settings.COMMON["lab_info"].values():
            for combo in settings.COMMON["window_log_name"]:
                self.assertTrue((audio / (info["pickup_audio_prefix"] + combo + ".wav")).is_file())
            for count in range(1, 4):
                self.assertTrue((audio / (info["delivery_audio_prefix"] + str(count) + ".wav")).is_file())
        for value in [0, 5, 6, 7, 8, 9, 10]:
            self.assertTrue((audio / ("WAIT-%d.wav" % value)).is_file())
        self.assertTrue((audio / "error.wav").is_file())
        self.assertEqual(len(list(Path(package_path("resources", "board2_templates")).glob("*.png"))), 7)

    @unittest.skipUnless(BASELINE.is_dir(), "Original workspace not shipped with standalone package")
    def test_original_package_unchanged_and_all_config_values_preserved(self):
        hashes = json.loads((PACKAGE / "docs/baseline_sha256.json").read_text(encoding="utf-8"))
        actual = {p.relative_to(BASELINE).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in BASELINE.rglob("*") if p.is_file()}
        self.assertEqual(actual, hashes)
        original = {}
        exec(compile((BASELINE / "scripts/dual_car_config.py").read_text(encoding="utf-8"),
                     "original_config", "exec"), original)
        self.assertEqual(settings.CARS, original["CARS"])
        self.assertEqual({k: v for k, v in settings.COMMON.items() if k != "paths"},
                         {k: v for k, v in original["COMMON"].items() if k != "paths"})

    @unittest.skipUnless(BASELINE.is_dir(), "Original workspace not shipped with standalone package")
    def test_business_function_bodies_preserved(self):
        # Check every original function, including class methods. Only explicit
        # dependency injection and moving entry guards are allowed exceptions.
        new_functions = {}
        for path in LIBRARY.rglob("*.py"):
            for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
                if isinstance(node, ast.FunctionDef):
                    new_functions.setdefault(node.name, []).append(node)
        changed = {"main", "query_board2_tcp", "publish_board2_tcp_result", "detect_board2_status", "safe_int"}
        checked = 0
        for path in (BASELINE / "scripts").glob("*.py"):
            for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
                if isinstance(node, ast.FunctionDef) and node.name not in changed:
                    self.assertIn(node.name, new_functions, (path.name, node.name))
                    self.assertIn(ast.dump(node), [ast.dump(n) for n in new_functions[node.name]],
                                  (path.name, node.name))
                    checked += 1
        self.assertGreater(checked, 120)
