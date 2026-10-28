"""Ensure damaged or incomplete release evidence fails the CI integrity gate."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from verify_r14_release_manifest import REQUIRED, verify


class ReleaseIntegrity(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.package = self.root / 'exports'
        self.package.mkdir()
        self.review = self.root / 'review'
        self.review.mkdir()
        def record(base, name, content):
            (base / name).write_bytes(content)
            return {'path': name, 'bytes': len(content),
                    'sha256': hashlib.sha256(content).hexdigest()}
        self.data = {'files': [record(self.package, n, b'[]' if n.startswith('NATIVE_') else b'original')
                               for n in sorted(REQUIRED)],
                     'reviewDirectory': '../review',
                     'reviewFiles': [record(self.review, 'evidence.json', b'{}')]}
        self.manifest = self.package / 'RELEASE_MANIFEST.json'

    def check(self):
        self.manifest.write_text(json.dumps(self.data), encoding='utf8')
        return verify(self.manifest, self.root)

    def test_complete_evidence_passes(self):
        self.assertEqual(self.check()['reviewFiles'], 1)

    def test_same_size_corruption_is_rejected(self):
        (self.package / 'R14-project.epro2').write_bytes(b'modified')
        with self.assertRaisesRegex(ValueError, 'integrity mismatch'):
            self.check()

    def test_missing_review_file_is_rejected(self):
        (self.review / 'evidence.json').unlink()
        with self.assertRaises(FileNotFoundError):
            self.check()

    def test_dropped_required_artifact_is_rejected(self):
        self.data['files'] = [r for r in self.data['files'] if r['path'] != 'R14-CPL-JLC.csv']
        with self.assertRaisesRegex(ValueError, 'Incomplete'):
            self.check()

    def test_path_escape_is_rejected(self):
        self.data['reviewFiles'][0]['path'] = '../../outside.json'
        with self.assertRaisesRegex(ValueError, 'Escaping'):
            self.check()


if __name__ == '__main__':
    unittest.main()
