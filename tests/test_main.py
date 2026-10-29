import unittest
from io import StringIO
from unittest.mock import patch

from kinova_movement_app.main import main


class MainTests(unittest.TestCase):
    def test_main_prints_ready_message(self) -> None:
        output = StringIO()
        with patch("sys.stdout", output):
            main()

        self.assertEqual(output.getvalue(), "Kinova Movement App is ready.\n")


if __name__ == "__main__":
    unittest.main()
