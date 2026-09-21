FRONT_SPLIT = 0.5
SIDE_MIN = 0.15
SIDE_MAX = 0.85


def assign_slot(face, detection):
    nx = float(detection.nx)
    if face == "front":
        return "front_left" if nx < FRONT_SPLIT else "front_right"
    if face == "left":
        if SIDE_MIN <= nx <= SIDE_MAX:
            return "inner_left"
        return None
    if face == "right":
        if SIDE_MIN <= nx <= SIDE_MAX:
            return "inner_right"
        return None
    return None
