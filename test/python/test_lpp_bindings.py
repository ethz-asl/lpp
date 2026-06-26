import logging

import pytest

from lpp import LogMode, Logger, LppHandler, LppSeverity


LEVEL_CASES = [
    (logging.DEBUG, "debug", "DEBUG ", LppSeverity.D),
    (logging.INFO, "info", "INFO  ", LppSeverity.I),
    (logging.WARNING, "warning", "WARN  ", LppSeverity.W),
    (logging.ERROR, "error", "ERROR ", LppSeverity.E),
    (logging.CRITICAL, "critical", "FATAL ", LppSeverity.F),
]


def make_logger(handler, name="test-lpp"):
    logger = logging.getLogger(name)
    logger.handlers.clear()
    logger.propagate = False
    logger.setLevel(logging.DEBUG)
    logger.addHandler(handler)
    return logger


def test_import_smoke():
    assert Logger is not None
    assert LppHandler is not None
    assert LogMode.MODE_LPP is not None
    assert LppSeverity.I is not None


def test_logger_factory_returns_standard_logger():
    logger = Logger("test-lpp-factory-standard")

    assert isinstance(logger, logging.Logger)


def test_logger_factory_uses_lpp_format(capfd):
    logger = Logger("test-lpp-factory-format")

    logger.info("Test123")

    assert capfd.readouterr().out == "INFO  Test123\n"


def test_logger_factory_accepts_mode(capfd):
    logger = Logger("test-lpp-factory-nolog", LogMode.MODE_NOLOG)

    logger.info("Test123")

    assert capfd.readouterr().out == ""


def test_logger_factory_default_level_is_info(capfd):
    logger = Logger("test-lpp-factory-default-level")

    logger.debug("Hidden")
    logger.info("Visible")

    assert capfd.readouterr().out == "INFO  Visible\n"


def test_logger_factory_accepts_level(capfd):
    logger = Logger("test-lpp-factory-debug-level", level=logging.DEBUG)

    logger.debug("Visible")

    assert capfd.readouterr().out == "DEBUG Visible\n"


def test_logger_factory_is_idempotent(capfd):
    logger = Logger("test-lpp-factory-idempotent")
    same_logger = Logger("test-lpp-factory-idempotent")

    assert same_logger is logger

    logger.info("Test123")

    assert capfd.readouterr().out == "INFO  Test123\n"


def test_logger_factory_preserves_user_handlers(capfd):
    entries = []

    class ListHandler(logging.Handler):
        def emit(self, record):
            entries.append(record.getMessage())

    logger = logging.getLogger("test-lpp-factory-user-handler")
    logger.handlers.clear()
    logger.addHandler(ListHandler())

    Logger("test-lpp-factory-user-handler")
    logger.info("Test123")

    assert capfd.readouterr().out == "INFO  Test123\n"
    assert entries == ["Test123"]


def test_logger_factory_callback(capfd):
    entries = []
    logger = Logger(
        "test-lpp-factory-callback",
        callback=lambda severity, message: entries.append((severity, message)),
    )

    logger.warning("Test%s", 123)

    assert capfd.readouterr().out == ""
    assert entries == [(LppSeverity.W, "Test123")]


def test_logger_factory_sysd_sender(capfd):
    entries = []

    def sender(severity, message, identifier):
        entries.append((severity, message, identifier))

    logger = Logger(
        "test-lpp-factory-sysd",
        LogMode.MODE_SYSD,
        identifier="pytest-lpp",
        sysd_sender=sender,
    )

    logger.info("Test%s", 123)

    assert capfd.readouterr().out == ""
    assert entries == [(LppSeverity.I, "Test123", "pytest-lpp")]


@pytest.mark.parametrize(("level", "method_name", "prefix", "_severity"), LEVEL_CASES)
def test_lpp_handler_outputs_lpp_format(capfd, level, method_name, prefix, _severity):
    logger = make_logger(LppHandler(), name=f"test-lpp-{level}")

    getattr(logger, method_name)("Test123")

    assert capfd.readouterr().out == f"{prefix}Test123\n"


def test_default_mode_uses_lpp_format(capfd):
    logger = make_logger(LppHandler(LogMode.MODE_DEFAULT), name="test-lpp-default")

    logger.info("Test123")

    assert capfd.readouterr().out == "INFO  Test123\n"


def test_python_percent_formatting(capfd):
    logger = make_logger(LppHandler(), name="test-lpp-formatting")

    logger.warning("Base angle (%f) is less than the minimum angle (%f)", 3.3, 5.5)

    assert capfd.readouterr().out == "WARN  Base angle (3.300000) is less than the minimum angle (5.500000)\n"


def test_formatter_is_respected(capfd):
    handler = LppHandler()
    handler.setFormatter(logging.Formatter("%(levelname)s:%(message)s"))
    logger = make_logger(handler, name="test-lpp-formatter")

    logger.warning("Test%s", 123)

    assert capfd.readouterr().out == "WARN  WARNING:Test123\n"


@pytest.mark.parametrize(("level", "method_name", "_prefix", "severity"), LEVEL_CASES)
def test_callback_receives_mapped_severity(capfd, level, method_name, _prefix, severity):
    entries = []
    handler = LppHandler(callback=lambda callback_severity, message: entries.append((callback_severity, message)))
    logger = make_logger(handler, name=f"test-lpp-callback-{level}")

    getattr(logger, method_name)("Test%s", 123)

    assert capfd.readouterr().out == ""
    assert entries == [(severity, "Test123")]


def test_custom_low_level_maps_to_debug(capfd):
    entries = []
    handler = LppHandler(callback=lambda severity, message: entries.append((severity, message)))
    logger = make_logger(handler, name="test-lpp-custom-low")
    logger.setLevel(1)

    logger.log(5, "Test123")

    assert capfd.readouterr().out == ""
    assert entries == [(LppSeverity.D, "Test123")]


def test_nolog_mode_emits_nothing(capfd):
    logger = make_logger(LppHandler(LogMode.MODE_NOLOG), name="test-lpp-nolog")

    for _level, method_name, _prefix, _severity in LEVEL_CASES:
        getattr(logger, method_name)("Test123")

    assert capfd.readouterr().out == ""


def test_sysd_mode_can_use_test_sender(capfd):
    entries = []

    def sender(severity, message, identifier):
        entries.append((severity, message, identifier))

    logger = make_logger(
        LppHandler(LogMode.MODE_SYSD, identifier="pytest-lpp", sysd_sender=sender),
        name="test-lpp-sysd",
    )

    logger.info("Test%s", 123)

    assert capfd.readouterr().out == ""
    assert entries == [(LppSeverity.I, "Test123", "pytest-lpp")]


@pytest.mark.parametrize("mode", [LogMode.MODE_GLOG, LogMode.MODE_ROSLOG])
def test_external_backends_raise_clear_error(mode):
    with pytest.raises(RuntimeError):
        LppHandler(mode)
