#!/usr/bin/env python3
"""Dynamic mock node isolation verification.

Runs each mock node as a subprocess in an isolated ROS environment and verifies:
  - competition_mode=true  -> non-zero exit, refusal message
  - competition_mode=false -> passes guard, publishes "TEST MODE ONLY"

The test creates its own roscore on an auto-selected free port so that
it never connects to or interferes with any pre-existing ROS Master.
"""

from __future__ import annotations

import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"

MOCK_NODES = [
    "mock_localization_node",
    "mock_perception_node",
    "mock_mapping_node",
    "mock_planning_node",
]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _pick_free_port(host: str = "127.0.0.1") -> int:
    """Allocate a free TCP port on *host* and return it."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, 0))
        return s.getsockname()[1]


def _wait_for_master(master_uri: str, roscore_pid: int, timeout: float = 15.0) -> bool:
    """Block until *master_uri* responds to ``rostopic list``, but ONLY while
    *roscore_pid* is still alive.  Returns ``True`` when ready."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        # PID check first — prevents connecting to a pre-existing Master
        try:
            os.kill(roscore_pid, 0)
        except OSError:
            return False  # our roscore died
        try:
            subprocess.run(
                ["rostopic", "list"],
                env={**os.environ, "ROS_MASTER_URI": master_uri},
                capture_output=True, text=True, timeout=5.0,
            )
            return True
        except (subprocess.TimeoutExpired, Exception):
            time.sleep(0.5)
    return False


# ---------------------------------------------------------------------------
# Test case
# ---------------------------------------------------------------------------

class TestMockDynamicIsolation(unittest.TestCase):
    """Dynamic verification of mock-node guard behaviour in an isolated
    ROS environment."""

    # Class-level state set up once per module load
    _roscore_pid: int | None = None
    _master_uri: str = ""
    _ros_home: str = ""

    @classmethod
    def setUpClass(cls):
        """Create an isolated ROS Master on a free port."""
        uid = os.getuid()
        pid = os.getpid()

        # --- port ---
        cls._port = _pick_free_port()
        cls._master_uri = f"http://127.0.0.1:{cls._port}"

        # --- isolated ROS_HOME ---
        cls._ros_home = tempfile.mkdtemp(
            prefix=f"simenv-pr7-mock-ros-home-{uid}-{pid}-"
        )

        # --- start our own roscore ---
        roscore_proc = subprocess.Popen(
            ["roscore", "-p", str(cls._port)],
            env={**os.environ, "ROS_HOME": cls._ros_home},
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        cls._roscore_pid = roscore_proc.pid

        if not _wait_for_master(cls._master_uri, cls._roscore_pid, timeout=15.0):
            cls._teardown_roscore()
            raise RuntimeError(
                f"Failed to start isolated roscore on port {cls._port} "
                f"(PID={cls._roscore_pid})"
            )

        print(f"[setUpClass] Isolated roscore ready: PID={cls._roscore_pid}, "
              f"ROS_MASTER_URI={cls._master_uri}")

    @classmethod
    def _teardown_roscore(cls):
        """Kill ONLY our own roscore process group."""
        pid = cls._roscore_pid
        if pid is None:
            return
        try:
            os.kill(pid, 0)
        except OSError:
            cls._roscore_pid = None
            return
        # Graceful first
        try:
            os.killpg(os.getpgid(pid), signal.SIGTERM)
        except OSError:
            pass
        time.sleep(0.5)
        # Force if still alive
        try:
            os.kill(pid, 0)
            os.killpg(os.getpgid(pid), signal.SIGKILL)
        except OSError:
            pass
        # Reap
        try:
            os.waitpid(pid, os.WNOHANG)
        except OSError:
            pass
        cls._roscore_pid = None
        # Clean up temp dir
        p = cls._ros_home
        if p and os.path.isdir(p):
            import shutil
            shutil.rmtree(p, ignore_errors=True)

    @classmethod
    def tearDownClass(cls):
        cls._teardown_roscore()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _isolated_env(self) -> dict:
        """Return an environment dict that targets our isolated Master."""
        env = os.environ.copy()
        env["ROS_MASTER_URI"] = self._master_uri
        env["ROS_HOME"] = self._ros_home
        env["PYTHONUNBUFFERED"] = "1"
        return env

    def _run_node(self, script_name: str, competition_mode: str,
                  timeout_sec: float = 10.0) -> subprocess.CompletedProcess:
        """Run a single mock node under our isolated Master.

        Parameters
        ----------
        script_name : str
            Basename of the node script inside ``SCRIPTS_DIR``.
        competition_mode : str
            ``"true"`` or ``"false"`` — passed both as a ROS param and as
            the ``_competition_mode:=…`` command-line remap.
        timeout_sec : float
            Hard timeout for the subprocess.

        Returns
        -------
        subprocess.CompletedProcess
        """
        script_path = str(SCRIPTS_DIR / script_name)
        env = self._isolated_env()

        # 1. Set the global ROS parameter using a parameter array (no shell).
        subprocess.run(
            ["rosparam", "set", "/competition_loop/competition_mode", competition_mode],
            env=env, capture_output=True, text=True, timeout=5.0,
        )

        # 2. Launch the node with an explicit timeout wrapper.
        cmd = [
            "timeout", str(timeout_sec),
            sys.executable, script_path,
            f"_competition_mode:={competition_mode}",
        ]
        return subprocess.run(cmd, env=env, capture_output=True, text=True,
                              timeout=timeout_sec + 5)

    def _report_node_result(self, script_name: str,
                             result: subprocess.CompletedProcess) -> None:
        """Print full diagnostics for a node run (always visible in log)."""
        print(f"  [{script_name}]")
        print(f"    command:       {' '.join(result.args)}")
        print(f"    returncode:    {result.returncode}")
        print(f"    ROS_MASTER_URI: {self._master_uri}")
        stdout = (result.stdout or "").strip()
        stderr = (result.stderr or "").strip()
        if stdout:
            for line in stdout.splitlines():
                print(f"    [stdout] {line}")
        else:
            print("    [stdout] (empty)")
        if stderr:
            for line in stderr.splitlines():
                print(f"    [stderr] {line}")
        else:
            print("    [stderr] (empty)")

    # ------------------------------------------------------------------
    # Competition-mode checks
    # ------------------------------------------------------------------

    def _test_rejects_in_competition(self, script_name: str):
        """Verify node exits non-zero when competition_mode=true."""
        result = self._run_node(script_name, "true")
        self._report_node_result(script_name, result)
        combined = result.stdout + result.stderr

        self.assertNotEqual(
            result.returncode, 0,
            f"{script_name}: must exit non-zero in competition mode. "
            f"exit={result.returncode}\n"
            f"stdout={result.stdout[:500]}\nstderr={result.stderr[:500]}"
        )
        self.assertIn(
            "MUST NOT run", combined,
            f"{script_name}: must log refusal reason.\n"
            f"stdout={result.stdout[:500]}\nstderr={result.stderr[:500]}"
        )

    def _test_runs_in_test_mode(self, script_name: str):
        """Verify node passes guard when competition_mode=false."""
        script_path = str(SCRIPTS_DIR / script_name)
        env = self._isolated_env()

        # Set param
        subprocess.run(
            ["rosparam", "set", "/competition_loop/competition_mode", "false"],
            env=env, capture_output=True, text=True, timeout=5.0,
        )

        # Launch node (no timeout — we manage the lifecycle)
        cmd = [sys.executable, script_path, "_competition_mode:=false"]
        proc = subprocess.Popen(
            cmd, env=env,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        node_pid = proc.pid
        print(f"  [{script_name}] started (PID={node_pid})")
        print(f"    command:       {' '.join(cmd)}")
        print(f"    ROS_MASTER_URI: {self._master_uri}")

        # Poll for "TEST MODE ONLY" — up to 10 s
        found = False
        deadline = time.monotonic() + 10.0
        stdout_chunks: list[str] = []
        stderr_chunks: list[str] = []
        while time.monotonic() < deadline:
            # Still alive?
            alive = (proc.poll() is None)
            # Drain pipes
            try:
                import selectors
                sel = selectors.DefaultSelector()
                sel.register(proc.stdout, selectors.EVENT_READ)
                sel.register(proc.stderr, selectors.EVENT_READ)
                for key, _ in sel.select(timeout=0.3):
                    line = key.fileobj.readline()
                    if line:
                        if key.fileobj is proc.stdout:
                            stdout_chunks.append(line)
                        else:
                            stderr_chunks.append(line)
                sel.close()
            except Exception:
                pass

            combined = "".join(stdout_chunks + stderr_chunks)
            if "TEST MODE ONLY" in combined:
                found = True
                break
            if not alive:
                break

        # Kill the node (it's our process)
        try:
            os.kill(node_pid, signal.SIGTERM)
        except OSError:
            pass
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            try:
                os.kill(node_pid, signal.SIGKILL)
            except OSError:
                pass
            proc.wait()

        full_output = "".join(stdout_chunks + stderr_chunks)
        print(f"    [stdout+stderr] {full_output[:500]}")

        # Assertions
        if "MUST NOT run" in full_output:
            self.fail(
                f"{script_name}: incorrectly refused in test mode. "
                f"output={full_output[:400]}"
            )
        if not found:
            self.fail(
                f"{script_name}: TEST MODE ONLY not found in output within 10 s. "
                f"output={full_output[:400]}"
            )

    # ------------------------------------------------------------------
    # The 8 test cases
    # ------------------------------------------------------------------

    def test_localization_rejects_competition(self):
        self._test_rejects_in_competition("mock_localization_node")

    def test_localization_runs_test(self):
        self._test_runs_in_test_mode("mock_localization_node")

    def test_perception_rejects_competition(self):
        self._test_rejects_in_competition("mock_perception_node")

    def test_perception_runs_test(self):
        self._test_runs_in_test_mode("mock_perception_node")

    def test_mapping_rejects_competition(self):
        self._test_rejects_in_competition("mock_mapping_node")

    def test_mapping_runs_test(self):
        self._test_runs_in_test_mode("mock_mapping_node")

    def test_planning_rejects_competition(self):
        self._test_rejects_in_competition("mock_planning_node")

    def test_planning_runs_test(self):
        self._test_runs_in_test_mode("mock_planning_node")


if __name__ == "__main__":
    unittest.main()
