import unittest
from unittest.mock import MagicMock, patch

from kinova_movement_app.main import main


class MainTests(unittest.TestCase):
    @patch("kinova_movement_app.main.MainWindow")
    @patch("kinova_movement_app.main.QApplication")
    def test_main_shows_the_window_and_starts_event_loop(
        self, application_class: MagicMock, window_class: MagicMock
    ) -> None:
        application = application_class.return_value
        application.exec.return_value = 0

        result = main()

        window_class.return_value.show.assert_called_once_with()
        application.exec.assert_called_once_with()
        self.assertEqual(result, 0)


if __name__ == "__main__":
    unittest.main()
