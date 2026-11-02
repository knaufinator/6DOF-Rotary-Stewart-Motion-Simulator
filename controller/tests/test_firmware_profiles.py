"""Exercise real CMake profile gates without an SDK, compiler, serial port or device."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
CMAKE = os.environ.get('CMAKE_COMMAND') or shutil.which('cmake')


class FirmwareProfiles(unittest.TestCase):
    def invoke(self, directory, *definitions):
        env = os.environ.copy()
        env.pop('IDF_TARGET', None)
        return subprocess.run([CMAKE, *('-D'+d for d in definitions), '-P',
                               str(ROOT/'cmake/FirmwareProfile.cmake')],
                              cwd=directory, env=env, text=True, capture_output=True)

    def test_profiles_select_expected_chips_and_safety(self):
        for profile, chip in [('mini', 'esp32'), ('legacy', 'esp32'),
                              ('r13-bench', 'esp32s3'), ('r14', 'esp32s3')]:
            with self.subTest(profile=profile), tempfile.TemporaryDirectory() as tmp:
                result = self.invoke(tmp, 'FIRMWARE_PROFILE='+profile, 'IDF_TARGET='+chip)
                self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
                self.assertEqual((Path(tmp)/'controller-profile.txt').read_text(), profile)

    def test_wrong_chip_unknown_profile_and_unsafe_flags_are_rejected(self):
        cases = [(['FIRMWARE_PROFILE=mini', 'IDF_TARGET=esp32s3'], 'requires esp32'),
                 (['FIRMWARE_PROFILE=r14', 'IDF_TARGET=esp32'], 'requires esp32s3'),
                 (['FIRMWARE_PROFILE=typo'], 'Unknown FIRMWARE_PROFILE'),
                 (['FIRMWARE_PROFILE=r14', 'R14_FAIL_CLOSED=OFF'], 'R14_FAIL_CLOSED=ON'),
                 (['FIRMWARE_PROFILE=r14', 'R14_BOARD_IO=OFF'], 'R14_BOARD_IO=ON'),
                 (['FIRMWARE_PROFILE=r14', 'R13_BENCH_ONE_AXIS=ON'], 'R13_BENCH_ONE_AXIS=OFF'),
                 (['FIRMWARE_PROFILE=mini', 'R14_BOARD_IO=ON'], 'R14_BOARD_IO=OFF')]
        for definitions, error in cases:
            with self.subTest(definitions=definitions), tempfile.TemporaryDirectory() as tmp:
                result = self.invoke(tmp, *definitions)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(error, result.stdout+result.stderr)

    def test_profile_switch_requires_a_new_build_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(self.invoke(tmp, 'FIRMWARE_PROFILE=mini').returncode, 0)
            result = self.invoke(tmp, 'FIRMWARE_PROFILE=legacy')
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('use a separate build directory', result.stderr)


if __name__ == '__main__':
    if not CMAKE:
        raise SystemExit('CMake is required for profile contract tests')
    unittest.main()
