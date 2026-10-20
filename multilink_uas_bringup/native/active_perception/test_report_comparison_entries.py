import unittest
from report_gimbal_improvement import comparison_entries


class ComparisonEntries(unittest.TestCase):
    def test_nonflight_diagnostics_are_not_accuracy_trials(self):
        self.assertEqual(comparison_entries({'seed': 42, 'cases': [], 'valid': True}), [])

    def test_failed_trial_remains_in_history(self):
        row = {'valid': False, 'checks': {'complete': False}}
        self.assertEqual(comparison_entries({'run1_prefix_active': row}), [('run1_prefix_active', row)])

    def test_malformed_declared_validation_is_not_silently_ignored(self):
        for result in [[], {'seed': 42}, {'run1_prefix_active': 1},
                       {'run1_prefix_active': {'valid': True}}]:
            with self.assertRaises(ValueError):
                comparison_entries(result, required=True)


if __name__ == '__main__':
    unittest.main()
