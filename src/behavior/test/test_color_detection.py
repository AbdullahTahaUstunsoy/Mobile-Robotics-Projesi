import cv2
import numpy as np
from behavior.color_detection import detect

# Colors sampled from the Gazebo camera (BGR)
FOLIAGE = (47, 83, 47)
FOLIAGE_LIT = (60, 103, 60)
PLANTER = (52, 68, 83)
WALL = (128, 122, 122)
RED_APPLE = (25, 25, 112)
GREEN_APPLE = (47, 106, 73)
RED_PANEL = (30, 30, 200)
GREEN_PANEL = (49, 137, 33)


def scene():
    img = np.zeros((480, 640, 3), np.uint8)
    img[:] = FOLIAGE
    img[:, 320:] = FOLIAGE_LIT
    img[360:] = PLANTER
    img[200:280, 280:360] = WALL
    return img


def apple(img, color, center=(320, 175), radius=30):
    cv2.circle(img, center, radius, color, -1)
    return img


def panel(img, color, corners=((260, 120), (380, 120), (380, 300), (260, 300))):
    cv2.fillPoly(img, [np.array(corners, np.int32)], color)
    return img


def test_background_ignored():
    assert detect(scene())[0] == {}


def test_red_and_green_apples():
    dets, _ = detect(apple(apple(scene(), RED_APPLE, (100, 240)), GREEN_APPLE, (540, 240)))
    assert set(dets) == {"red_apple", "green_apple"}
    assert dets["red_apple"].offset_x < -0.5
    assert dets["green_apple"].offset_x > 0.5


def test_red_and_green_checkpoints():
    img = panel(scene(), RED_PANEL, ((40, 100), (200, 100), (200, 300), (40, 300)))
    img = panel(img, GREEN_PANEL, ((440, 100), (600, 100), (600, 300), (440, 300)))
    dets, _ = detect(img)
    assert set(dets) == {"red_checkpoint", "green_checkpoint"}
    assert dets["red_checkpoint"].offset_x < 0 < dets["green_checkpoint"].offset_x


def test_checkpoint_seen_at_an_angle():
    # A panel on a side wall shows up as a trapezoid
    dets, _ = detect(panel(scene(), GREEN_PANEL, ((400, 150), (500, 110), (500, 330), (400, 290))))
    assert set(dets) == {"green_checkpoint"}


def test_apple_next_to_checkpoint_of_same_color():
    img = panel(scene(), RED_PANEL, ((380, 100), (560, 100), (560, 300), (380, 300)))
    dets, _ = detect(apple(img, RED_APPLE, (150, 240)))
    assert set(dets) == {"red_apple", "red_checkpoint"}


def test_largest_blob_per_label():
    img = apple(apple(scene(), RED_APPLE, (100, 240), radius=15), RED_APPLE, (500, 240), radius=40)
    assert detect(img)[0]["red_apple"].offset_x > 0


def test_tiny_blob_ignored():
    assert detect(apple(scene(), RED_APPLE, radius=4))[0] == {}
