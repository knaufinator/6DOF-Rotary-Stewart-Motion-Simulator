"""Regression: supplier subassemblies need not contain EasyEDA REF->0 children."""
import unittest
from validate_r14_exports import step_component_refs


def occurrence(ref, parent, child):
    return f"NEXT_ASSEMBLY_USAGE_OCCURRENCE('{ref}','','',#{parent},#{child},$);"


class StepReferences(unittest.TestCase):
    def test_supplier_subassembly_without_arrow_child_is_counted(self):
        text = occurrence('U1', 5, 10)+occurrence('U1->0', 10, 11)+occurrence('J3', 5, 20)+occurrence('', 20, 21)
        self.assertEqual(set(step_component_refs(text, {'U1', 'J3'})), {'U1', 'J3'})

    def test_missing_component_fails(self):
        with self.assertRaises(ValueError):
            step_component_refs(occurrence('U1', 5, 10), {'U1', 'J3'})

    def test_duplicate_component_fails(self):
        with self.assertRaises(ValueError):
            step_component_refs(occurrence('U1', 5, 10)*2+occurrence('J3', 5, 20), {'U1', 'J3'})

    def test_two_complete_roots_fail(self):
        with self.assertRaises(ValueError):
            step_component_refs(occurrence('U1', 5, 10)+occurrence('U1', 6, 20), {'U1'})


if __name__ == '__main__':
    unittest.main()
