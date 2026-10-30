"""Application entry point."""

import sys

from PySide6.QtWidgets import QApplication

from kinova_movement_app.layout import MainWindow


def main() -> int:
    """Start the Qt desktop application."""
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    main()
