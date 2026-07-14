"""Ana pencere paketi: dış dünyaya sadece MainWindow'u sunar (main.py
`from ui.main_window import MainWindow` şeklinde kullanır, davranış
değişmez)."""

from .window import MainWindow

__all__ = ["MainWindow"]
