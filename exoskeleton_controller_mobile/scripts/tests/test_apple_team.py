import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("apple_team", Path(__file__).parents[1] / "detect_apple_team.py")
teams = importlib.util.module_from_spec(spec)
spec.loader.exec_module(teams)


class TeamSelectionTests(unittest.TestCase):
    def test_signed_in_team_with_valid_identity(self):
        result = teams.choose_team({"TEAMAAAA01": "A", "TEAMBBBB02": "B"}, {"TEAMBBBB02": "Identity"}, {})
        self.assertEqual(result, "TEAMBBBB02")

    def test_single_account_without_certificate(self):
        self.assertEqual(teams.choose_team({"TEAMAAAA01": "A"}, {}, {}), "TEAMAAAA01")

    def test_certificate_or_profile_fallback(self):
        self.assertEqual(teams.choose_team({}, {"TEAMAAAA01": "A"}, {}), "TEAMAAAA01")
        self.assertEqual(teams.choose_team({}, {}, {"TEAMBBBB02": "B"}), "TEAMBBBB02")

    def test_no_guess_when_multiple_or_missing(self):
        with self.assertRaisesRegex(ValueError, "Multiple signing teams"):
            teams.choose_team({"TEAMAAAA01": "A", "TEAMBBBB02": "B"}, {}, {})
        with self.assertRaisesRegex(ValueError, "No Apple signing team"):
            teams.choose_team({}, {}, {})


if __name__ == "__main__":
    unittest.main()
