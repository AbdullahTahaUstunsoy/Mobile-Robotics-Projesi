"""Red/green apple and wall checkpoint detection with HSV masks. Pure OpenCV, no ROS, so it can be unit tested.

Color tells red from green; shape tells an apple from a checkpoint panel:
    apple       round blob   (a sphere seen from any side is a circle)
    checkpoint  4-corner convex blob (a flat panel is a rectangle, or a trapezoid seen at an angle)
"""

from dataclasses import dataclass

import cv2
import numpy as np

COLORS = ("red", "green")
KINDS = ("apple", "checkpoint")
LABELS = tuple(f"{color}_{kind}" for color in COLORS for kind in KINDS)


@dataclass
class HsvThresholds:
    # Red wraps around hue 0 in OpenCV's 0-179 range, so two hue bands are used
    red_hue_low_max: int = 10
    red_hue_high_min: int = 170
    red_sat_min: int = 120  # rejects brown planters (S ~95)
    green_hue_min: int = 35
    green_hue_max: int = 85
    green_sat_min: int = 130  # rejects foliage (S ~110) and the grey walls
    val_min: int = 50
DEFAULT_THRESHOLDS = HsvThresholds()

@dataclass
class Detection:
    label: str  # e.g. 'red_apple', 'green_checkpoint'
    cx: float  # centroid in pixels
    cy: float
    area: float  # contour area in pixels
    bbox: tuple  # x, y, w, h
    offset_x: float  # -1 (left edge) .. +1 (right edge)
    offset_y: float  # -1 (top edge) .. +1 (bottom edge)
    area_ratio: float  # fraction of the image covered by the blob


def _clean(mask):
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def color_masks(bgr, thresholds=DEFAULT_THRESHOLDS):
    """{'red': mask, 'green': mask}, binary uint8 (0/255) masks with small noise removed."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    t = thresholds
    red_low = cv2.inRange(hsv, (0, t.red_sat_min, t.val_min), (t.red_hue_low_max, 255, 255))
    red_high = cv2.inRange(hsv, (t.red_hue_high_min, t.red_sat_min, t.val_min), (179, 255, 255))
    green = cv2.inRange(
        hsv, (t.green_hue_min, t.green_sat_min, t.val_min), (t.green_hue_max, 255, 255)
    )
    return {"red": _clean(cv2.bitwise_or(red_low, red_high)), "green": _clean(green)}


def classify_shape(contour):
    """'checkpoint' for a 4-corner convex blob, 'apple' for a round one, None otherwise."""
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0:
        return None
    _, _, w, h = cv2.boundingRect(contour)
    extent = area / (w * h)  # circle ~0.79, upright rectangle ~1.0
    circularity = 4 * np.pi * area / perimeter**2  # circle 1.0, square 0.79
    corners = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
    if extent > 0.9 or (len(corners) == 4 and cv2.isContourConvex(corners)):
        return "checkpoint"
    if circularity > 0.6:
        return "apple"
    return None


def detect(bgr, thresholds=DEFAULT_THRESHOLDS, min_area=150.0):
    """Return ({label: largest Detection of that label}, masks). Labels that are not seen are left out."""
    masks = color_masks(bgr, thresholds)
    h, w = bgr.shape[:2]
    found = {}
    for color, mask in masks.items():
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            kind = classify_shape(contour)
            if kind is None:
                continue
            label = f"{color}_{kind}"
            if label in found and found[label].area >= area:
                continue
            m = cv2.moments(contour)
            cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
            found[label] = Detection(
                label=label,
                cx=cx,
                cy=cy,
                area=area,
                bbox=cv2.boundingRect(contour),
                offset_x=(cx - w / 2) / (w / 2),
                offset_y=(cy - h / 2) / (h / 2),
                area_ratio=area / (w * h),
            )
    return found, masks


# Box colors (BGR) for the debug image
_DRAW = {
    "red_apple": (0, 255, 255),
    "green_apple": (255, 255, 0),
    "red_checkpoint": (0, 0, 255),
    "green_checkpoint": (0, 255, 0),
}


def draw_debug(bgr, detections, masks):
    """Camera image with the detections drawn on it and the combined mask as an inset (top-right)."""
    out = bgr.copy()
    h, w = out.shape[:2]
    # Image center cross-hair
    cv2.line(out, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
    cv2.line(out, (0, h // 2), (w, h // 2), (255, 255, 255), 1)
    for det in detections.values():
        x, y, bw, bh = det.bbox
        color = _DRAW[det.label]
        cv2.rectangle(out, (x, y), (x + bw, y + bh), color, 2)
        cv2.circle(out, (int(det.cx), int(det.cy)), 4, color, -1)
        cv2.putText(
            out,
            det.label.replace("_", " ").upper(),
            (x, max(y - 6, 12)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
        )
    summary = ", ".join(sorted(detections)) if detections else "nothing detected"
    cv2.putText(out, summary, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    # Mask inset: red mask in red, green mask in green
    inset = np.zeros((h, w, 3), np.uint8)
    inset[masks["red"] > 0] = (0, 0, 255)
    inset[masks["green"] > 0] = (0, 255, 0)
    out[0 : h // 4, w - w // 4 : w] = cv2.resize(inset, (w // 4, h // 4))
    cv2.rectangle(out, (w - w // 4, 0), (w - 1, h // 4), (255, 255, 255), 1)
    return out
