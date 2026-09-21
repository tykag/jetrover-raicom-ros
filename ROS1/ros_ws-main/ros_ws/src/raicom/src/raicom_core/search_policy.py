from collections import namedtuple

SearchStep = namedtuple("SearchStep", "kind name dx")
SearchStep.__new__.__defaults__ = ("", 0.0)


class SearchPolicy(object):
    def __init__(self, lateral_m=0.12):
        lateral = max(0.10, min(0.15, float(lateral_m)))
        self.lateral_m = lateral
        self._steps = (
            SearchStep("pose", "look_high", 0.0),
            SearchStep("pose", "look_mid", 0.0),
            SearchStep("pose", "look_low", 0.0),
            SearchStep("translate", "left", -lateral),
            SearchStep("translate", "center", lateral),
            SearchStep("translate", "right", lateral),
            SearchStep("translate", "center", -lateral),
            SearchStep("home", "observe", 0.0),
        )

    def steps(self):
        return self._steps

    def can_continue_after(self, executed):
        return len(tuple(executed)) < len(self._steps)
