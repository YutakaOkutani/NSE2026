from mission.const import Phase

__all__ = ["CanSatController", "Phase"]


def __getattr__(name):
    if name == "CanSatController":
        from mission.ctrl import CanSatController

        return CanSatController
    raise AttributeError(f"module 'mission' has no attribute {name!r}")
