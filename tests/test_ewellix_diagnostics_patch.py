"""Exercise explicit installation in temporary local clones; never touch hardware."""

from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "ewellix" / "ewellix_lift"
INSTALLER = ROOT / "apply_ewellix_diagnostics_patch.sh"
PINNED = "eb41860fbdaefa5fa934551e54e65d384b81d59b"


class PatchInstallerTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="ewellix-patch-test-")
        self.addCleanup(self.temp.cleanup)
        self.checkout = Path(self.temp.name) / "source"
        subprocess.run(
            ["git", "clone", "--shared", "--quiet", str(SOURCE), str(self.checkout)],
            check=True,
        )
        self.git("checkout", "--quiet", "--detach", PINNED)

    def git(self, *args):
        return subprocess.run(
            ["git", "-C", str(self.checkout), *args],
            check=True, capture_output=True, text=True,
        ).stdout

    def install(self, *args):
        return subprocess.run(
            ["bash", str(INSTALLER), *args, "--source-dir", str(self.checkout)],
            capture_output=True, text=True,
        )

    def test_check_does_not_modify_and_apply_is_idempotent(self):
        checked = self.install("--check")
        self.assertEqual(checked.returncode, 0, checked.stderr)
        self.assertIn("status=ready", checked.stdout)
        self.assertEqual(self.git("status", "--porcelain"), "")
        self.assertEqual(self.install("--apply").returncode, 0)
        original_diff = self.git("diff")
        header = self.checkout / "ewellix_driver/include/ewellix_driver/ewellix_node/communication_state.hpp"
        self.assertTrue(header.is_file())
        self.assertIn("diagnostic_msgs", original_diff)
        applied = self.install("--apply")
        self.assertEqual(applied.returncode, 0, applied.stderr)
        self.assertIn("status=applied", applied.stdout)
        self.assertEqual(self.git("diff"), original_diff)
        self.assertEqual(self.install("--check").returncode, 0)

    def test_revision_mismatch_is_rejected_without_changes(self):
        self.git("checkout", "--quiet", "--detach", f"{PINNED}^")
        result = self.install("--apply")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("revision mismatch", result.stderr)
        self.assertEqual(self.git("status", "--porcelain"), "")

    def test_dirty_target_is_preserved(self):
        path = self.checkout / "ewellix_driver/package.xml"
        original = path.read_text() + "\n<!-- local work -->\n"
        path.write_text(original)
        result = self.install("--apply")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("local changes", result.stderr)
        self.assertEqual(path.read_text(), original)

    def test_untracked_target_collision_is_preserved(self):
        path = self.checkout / "ewellix_driver/test/test_cycle2_data.cpp"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("// local work\n")
        result = self.install("--apply")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("local changes", result.stderr)
        self.assertEqual(path.read_text(), "// local work\n")

    def test_unrelated_changes_are_preserved(self):
        path = self.checkout / "operator_notes.txt"
        path.write_text("keep this\n")
        result = self.install("--apply")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(path.read_text(), "keep this\n")


if __name__ == "__main__":
    unittest.main()
