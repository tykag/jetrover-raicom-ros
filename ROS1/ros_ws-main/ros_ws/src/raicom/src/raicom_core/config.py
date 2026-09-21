import os

try:
    import yaml
except ImportError:
    yaml = None


DEFAULTS = {
    "enable_base_motion": False,
    "enable_arm_motion": False,
    "allow_high": False,
    "grasp_mode": "fixed",
    "aim_calibrated": False,
    "roi_calibrated": False,
    "closed_gripper": 350,
    "search_lateral_m": 0.12,
    "aims": {},
    "rois": {},
}


def load_config(path):
    data = dict(DEFAULTS)
    if path and os.path.isfile(path):
        if yaml is None:
            raise RuntimeError("PyYAML is required to load %s" % path)
        with open(path, "r") as handle:
            loaded = yaml.safe_load(handle) or {}
        if not isinstance(loaded, dict):
            raise ValueError("config must be a mapping")
        data.update(loaded)
    return data


def refuse_reason(config):
    if str(config.get("grasp_mode", "fixed")).lower() != "fixed":
        return "grasp_mode must stay fixed"
    if bool(config.get("allow_high", False)):
        return "allow_high is disabled until high poses are taught"
    if not bool(config.get("aim_calibrated", False)):
        return "aim not calibrated"
    if not bool(config.get("roi_calibrated", False)):
        return "roi not calibrated"
    return None
