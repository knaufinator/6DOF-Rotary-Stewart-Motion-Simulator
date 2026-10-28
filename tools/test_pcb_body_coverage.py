"""Regression tests for the actual J12 outline missed by the original screen."""
import unittest
from pathlib import Path
from audit_pcb_mechanical import archive_parts, body_coverage
from verify_pcb_body_bounds import documents
from audit_pcb_outline_coverage import reconstruct

SOURCE = Path(__file__).resolve().parents[1] / 'hardware/pcb/6dof2_r14/exports/2026-10-26/R14-project.epro2'


class BodyCoverage(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        part = archive_parts(SOURCE)['J12']
        cls.rows = documents(SOURCE)['FOOTPRINT', part['footprintUuid']]

    def test_actual_j12_four_line_body_is_covered(self):
        result = body_coverage(self.rows)
        self.assertEqual(result['bodyLineSegments'], 4)
        self.assertTrue(result['bodyLineLoopsRecognized'])
        self.assertEqual(result['bodyOutlineState'], 'COVERED')

    def test_broken_j12_body_fails(self):
        rows = list(self.rows)
        index = next(i for i, (h, v) in enumerate(rows) if h['type'] == 'LINE' and v.get('layerId') == 48)
        del rows[index]
        self.assertEqual(body_coverage(rows)['bodyOutlineState'], 'UNVERIFIED')

    def test_missing_j12_body_fails(self):
        rows = [(h, v) for h, v in self.rows if v.get('layerId') != 48]
        self.assertEqual(body_coverage(rows)['bodyOutlineState'], 'UNVERIFIED')

    def test_independent_geometry_reconstructs_actual_j12(self):
        geometry, kinds = reconstruct(self.rows)
        self.assertEqual(kinds, {'LINE': 4})
        self.assertTrue(geometry.is_valid)
        self.assertGreater(geometry.area, 0)

    def test_independent_geometry_rejects_duplicate_edges(self):
        line = next(r for r in self.rows if r[0]['type'] == 'LINE' and r[1].get('layerId') == 48)
        with self.assertRaises(ValueError):
            reconstruct(self.rows + [line])

    def test_independent_geometry_rejects_self_crossing_polygon(self):
        with self.assertRaises(ValueError):
            reconstruct([({'type': 'POLY'}, {'layerId': 48, 'path': [0, 0, 'L', 1, 1, 0, 1, 1, 0, 0, 0]})])


if __name__ == '__main__':
    unittest.main()
