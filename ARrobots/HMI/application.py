"""Tk application construction and direct-run activation for the AR4 HMI."""

from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass
import logging
import os
import threading
from typing import Any

import tkinter as tk
import ttkbootstrap as ttk_bootstrap


@dataclass(frozen=True, eq=False)
class ApplicationShell:
    root: ttk_bootstrap.Tk
    tk_thread_id: int
    notebook: ttk_bootstrap.Notebook
    tab1: ttk_bootstrap.Frame
    tab2: ttk_bootstrap.Frame
    tab3: ttk_bootstrap.Frame
    tab4: ttk_bootstrap.Frame
    tab5: ttk_bootstrap.Frame
    tab6: ttk_bootstrap.Frame
    tab7: ttk_bootstrap.Frame
    tab8: ttk_bootstrap.Frame
    tab9: ttk_bootstrap.Frame


def create_application_shell(
    environment: Mapping[str, Any],
    logger: logging.Logger,
    icon_path: os.PathLike[str] | str,
) -> ApplicationShell:
    root = ttk_bootstrap.Tk()
    tk_thread_id = threading.get_ident()
    root.wm_title("AR4 Software Ver 6.7")
    root.iconphoto(True, tk.PhotoImage(file=os.fspath(icon_path)))

    if (
        environment["Platform"]["IS_RPI"]
        and environment["Platform"]["IS_HEADLESS"]
    ):
        rpi_scale = 0.75
        rpi_x_size = 1590
        rpi_y_size = 800
        logger.debug(
            "Running on headless Raspberry Pi - Adjusting scale to "
            f"{rpi_scale} and window size to {rpi_x_size}x{rpi_y_size}"
        )
        root.tk.call("tk", "scaling", rpi_scale)
        root.geometry(f"{rpi_x_size}x{rpi_y_size}+0+0")
    else:
        root.geometry("1600x900+0+0")

    root.resizable(width=True, height=True)

    notebook = ttk_bootstrap.Notebook(root)
    root.grid_rowconfigure(0, weight=1)
    root.grid_columnconfigure(0, weight=1)
    notebook.grid(row=0, column=0, sticky="nsew")

    tab1 = ttk_bootstrap.Frame(notebook)
    notebook.add(tab1, text=" Main Controls ")
    tab2 = ttk_bootstrap.Frame(notebook)
    notebook.add(tab2, text="  Config Settings  ")
    tab3 = ttk_bootstrap.Frame(notebook)
    notebook.add(tab3, text="   Kinematics    ")
    tab4 = ttk_bootstrap.Frame(notebook)
    notebook.add(tab4, text=" Inputs Outputs ")
    tab5 = ttk_bootstrap.Frame(notebook)
    notebook.add(tab5, text="   Registers    ")
    tab6 = ttk_bootstrap.Frame(notebook)
    notebook.add(tab6, text="   Vision    ")
    tab7 = ttk_bootstrap.Frame(notebook)
    notebook.add(tab7, text="    G-Code     ")
    tab8 = ttk_bootstrap.Frame(notebook)
    notebook.add(tab8, text="      Log      ")
    tab9 = ttk_bootstrap.Frame(notebook)

    return ApplicationShell(
        root=root,
        tk_thread_id=tk_thread_id,
        notebook=notebook,
        tab1=tab1,
        tab2=tab2,
        tab3=tab3,
        tab4=tab4,
        tab5=tab5,
        tab6=tab6,
        tab7=tab7,
        tab8=tab8,
        tab9=tab9,
    )


def run_application(
    root: ttk_bootstrap.Tk,
    startup_widget: ttk_bootstrap.Frame,
    event_poll_names: Iterable[str],
    schedule_event_poll: Callable[[str], bool],
    saved_connection_callback: Callable[[], Any],
) -> None:
    """Preserve observable poll order, then schedule connection and run Tk."""
    for poll_name in event_poll_names:
        schedule_event_poll(poll_name)
    startup_widget.after(100, saved_connection_callback)
    root.mainloop()
