"""Python logging integration for Log++."""

import logging
import os
import sys

from ._lpp import LogMode, LppSeverity, _LppEmitter


def _severity_from_level(levelno):
    if levelno >= logging.CRITICAL:
        return LppSeverity.F
    if levelno >= logging.ERROR:
        return LppSeverity.E
    if levelno >= logging.WARNING:
        return LppSeverity.W
    if levelno >= logging.INFO:
        return LppSeverity.I
    return LppSeverity.D


def _default_sysd_identifier():
    argv0 = sys.argv[0] if sys.argv else ""
    return os.path.basename(argv0)


def _resolve_identifier(mode, identifier):
    if mode == LogMode.MODE_SYSD and identifier is None:
        return _default_sysd_identifier()
    return identifier


class LppHandler(logging.Handler):
    """Logging handler that forwards Python log records to Log++."""

    def __init__(
        self,
        mode=LogMode.MODE_LPP,
        identifier=None,
        level=logging.NOTSET,
        callback=None,
        sysd_sender=None,
    ):
        super().__init__(level)
        self._emitter = _LppEmitter(
            mode=mode,
            identifier=_resolve_identifier(mode, identifier),
            callback=callback,
            sysd_sender=sysd_sender,
        )

    def emit(self, record):
        try:
            self._emitter.emit(_severity_from_level(record.levelno), self.format(record))
        except Exception:
            self.handleError(record)


def Logger(
    name,
    mode=LogMode.MODE_LPP,
    level=logging.INFO,
    identifier=None,
    callback=None,
    sysd_sender=None,
    propagate=False,
):
    """Return a standard logger configured with an LppHandler."""

    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.propagate = propagate

    for handler in list(logger.handlers):
        if isinstance(handler, LppHandler) and getattr(handler, "_installed_by_lpp_logger", False):
            logger.removeHandler(handler)

    handler = LppHandler(
        mode=mode,
        identifier=identifier,
        callback=callback,
        sysd_sender=sysd_sender,
    )
    handler._installed_by_lpp_logger = True
    logger.addHandler(handler)
    return logger
