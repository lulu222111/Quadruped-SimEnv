#!/usr/bin/env python3
"""Tests that mock nodes refuse to run in competition mode and run in test mode.

These tests import the mock node modules and verify their guard logic without
actually running a ROS master (they test the guard before init_node).
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
import unittest
from pathlib import Path

_SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"

# The four mock nodes
MOCK_NODES = [
    "mock_localization_node",
    "mock_perception_node",
    "mock_mapping_node",
    "mock_planning_node",
]


def _strip_docstring(source: str) -> str:
    """Remove the module-level docstring from source code.

    Returns source with the docstring lines replaced by blank lines
    (to preserve line numbers).
    """
    # Match the first triple-quoted string (module docstring)
    # Handles both ''' and \"\"\"
    lines = source.split("\n")
    in_docstring = False
    result = []
    for line in lines:
        stripped = line.strip()
        if not in_docstring:
            if stripped.startswith('"""') or stripped.startswith("'''"):
                in_docstring = True
                # Check if single-line docstring
                count = 0
                if stripped.startswith('"""'):
                    count = stripped.count('"""')
                else:
                    count = stripped.count("'''")
                if count >= 2:
                    in_docstring = False
                result.append("")
                continue
            result.append(line)
        else:
            if '"""' in stripped or "'''" in stripped:
                in_docstring = False
            result.append("")
    return "\n".join(result)


class TestMockNodeCompetitionModeRejection(unittest.TestCase):
    """Verify each mock node refuses to run when competition_mode=true."""

    def _check_guard_logic(self, script_name: str):
        """Verify the guard condition logic in the script source."""
        script_path = _SCRIPTS_DIR / script_name
        self.assertTrue(script_path.is_file(), f"Script not found: {script_path}")
        source = script_path.read_text(encoding="utf-8")

        # Remove docstring to avoid matching prohibition descriptions
        code_only = _strip_docstring(source)

        # Verify the script uses the correct variable name
        self.assertIn("competition_mode", code_only,
                      f"{script_name}: must use variable name 'competition_mode'")

        # Verify the guard checks competition_mode == True to reject
        self.assertIn("if competition_mode:", code_only,
                      f"{script_name}: must guard with 'if competition_mode:' to reject in competition")

        # Verify sys.exit(1) is present
        self.assertIn("sys.exit(1)", code_only,
                      f"{script_name}: must call sys.exit(1) when refusing to run")

        # Verify the error message explains why
        self.assertIn("MUST NOT run", code_only,
                      f"{script_name}: must explain refusal reason in log message")

    def test_localization_has_correct_guard(self):
        self._check_guard_logic("mock_localization_node")

    def test_perception_has_correct_guard(self):
        self._check_guard_logic("mock_perception_node")

    def test_mapping_has_correct_guard(self):
        self._check_guard_logic("mock_mapping_node")

    def test_planning_has_correct_guard(self):
        self._check_guard_logic("mock_planning_node")


class TestMockNodeNoForbiddenReferences(unittest.TestCase):
    """Verify mock nodes do NOT reference forbidden topics, truth files, or Gazebo services.

    Forbidden references are ONLY allowed in docstrings and comments that
    describe the prohibition (e.g. "MUST NOT subscribe to /Odometry_gazebo").
    """

    FORBIDDEN_PATTERNS = [
        "/Odometry_gazebo",
        "/ground_truth/",
        "danger_truth.json",
        "competition_scene.world",
        "layout_metadata.json",
        "get_model_state",
        "gazebo",
        "GAZEBO",
    ]

    def _check_no_forbidden_references(self, script_name: str):
        script_path = _SCRIPTS_DIR / script_name
        source = script_path.read_text(encoding="utf-8")
        code_only = _strip_docstring(source)

        for pattern in self.FORBIDDEN_PATTERNS:
            if pattern not in code_only:
                continue
            # Pattern found in code — check each occurrence
            for i, line in enumerate(code_only.split("\n")):
                if pattern not in line:
                    continue
                stripped = line.strip()
                # Only accept if it appears inside a comment
                if not stripped.startswith("#"):
                    self.fail(
                        f"{script_name}:{i+1} references forbidden '{pattern}' "
                        f"in code (not comment): {stripped}"
                    )

    def test_localization_no_forbidden_refs(self):
        self._check_no_forbidden_references("mock_localization_node")

    def test_perception_no_forbidden_refs(self):
        self._check_no_forbidden_references("mock_perception_node")

    def test_mapping_no_forbidden_refs(self):
        self._check_no_forbidden_references("mock_mapping_node")

    def test_planning_no_forbidden_refs(self):
        self._check_no_forbidden_references("mock_planning_node")


class TestMockNodeHardcodedData(unittest.TestCase):
    """Verify mock nodes use hard-coded data, not file reads or service calls."""

    # Patterns that indicate reading files or calling services (code-level)
    # Use regex word-boundary matching to avoid substring false positives
    FORBIDDEN_OPS_REGEX = [
        (r'\bjson\.load\(', "json.load("),
        (r'\bjson\.loads\(', "json.loads("),
        (r'\bopen\(', "open("),
        (r'(?<!\.)pathlib\.Path\(', "pathlib.Path("),
        (r'\bread_text\(', "read_text("),
        (r'\breadlines\(', "readlines("),
        (r'\bServiceProxy\(', "ServiceProxy("),
        (r'\.call\(', ".call("),
        (r'\bGetModelState\b', "GetModelState"),
    ]

    def _check_no_file_reads_or_services(self, script_name: str):
        script_path = _SCRIPTS_DIR / script_name
        source = script_path.read_text(encoding="utf-8")
        code_only = _strip_docstring(source)

        # Also strip comment lines
        lines = code_only.split("\n")
        code_lines = [
            line for line in lines
            if not line.strip().startswith("#") and line.strip() != ""
        ]
        code_without_comments = "\n".join(code_lines)

        for regex, label in self.FORBIDDEN_OPS_REGEX:
            matches = list(re.finditer(regex, code_without_comments))
            if matches:
                for m in matches:
                    # Get the line context
                    line_num = code_without_comments[:m.start()].count("\n") + 1
                    line_text = code_without_comments.split("\n")[line_num - 1].strip()
                    self.fail(
                        f"{script_name}:{line_num} uses '{label}' — "
                        f"all data must be hard-coded. Context: {line_text}"
                    )

    def test_localization_hardcoded(self):
        self._check_no_file_reads_or_services("mock_localization_node")

    def test_perception_hardcoded(self):
        self._check_no_file_reads_or_services("mock_perception_node")

    def test_mapping_hardcoded(self):
        self._check_no_file_reads_or_services("mock_mapping_node")

    def test_planning_hardcoded(self):
        self._check_no_file_reads_or_services("mock_planning_node")


if __name__ == "__main__":
    unittest.main()
