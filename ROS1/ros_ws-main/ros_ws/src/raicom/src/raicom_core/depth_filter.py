from raicom_core.types import DepthSample


def _percentile(sorted_vals, pct):
    if not sorted_vals:
        return 0.0
    if len(sorted_vals) == 1:
        return float(sorted_vals[0])
    k = (len(sorted_vals) - 1) * float(pct) / 100.0
    lo = int(k)
    hi = min(lo + 1, len(sorted_vals) - 1)
    frac = k - lo
    return sorted_vals[lo] * (1.0 - frac) + sorted_vals[hi] * frac


def filter_depth(values, min_valid=12, max_spread_m=0.035, lo=0.08, hi=2.5):
    valid = [float(v) for v in values if lo < float(v) < hi]
    if len(valid) < int(min_valid):
        return DepthSample(False, 0.0, len(valid), 0.0, "too_few")
    ordered = sorted(valid)
    near_limit = _percentile(ordered, 35)
    surface = [v for v in ordered if v <= near_limit] or ordered
    spread = _percentile(surface, 90) - _percentile(surface, 10)
    if spread > float(max_spread_m):
        return DepthSample(False, 0.0, len(valid), spread, "spread")
    mid = surface[len(surface) // 2]
    return DepthSample(True, float(mid), len(valid), spread, "ok")
