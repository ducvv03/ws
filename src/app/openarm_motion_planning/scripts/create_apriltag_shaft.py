import cv2
import numpy as np
from pathlib import Path


TAG_ID = 5
MARKER_SIZE_PX = 800
OUTPUT_PATH = "apriltag_shaft_id10.png"


def main():
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_APRILTAG_36h11
    )

    marker_img = np.zeros(
        (MARKER_SIZE_PX, MARKER_SIZE_PX),
        dtype=np.uint8
    )

    cv2.aruco.drawMarker(
        dictionary,
        TAG_ID,
        MARKER_SIZE_PX,
        marker_img,
        1
    )

    margin = 120

    canvas = np.ones(
        (MARKER_SIZE_PX + 2 * margin, MARKER_SIZE_PX + 2 * margin),
        dtype=np.uint8
    ) * 255

    canvas[
        margin:margin + MARKER_SIZE_PX,
        margin:margin + MARKER_SIZE_PX
    ] = marker_img

    cv2.imwrite(OUTPUT_PATH, canvas)

    print(f"Saved: {Path(OUTPUT_PATH).resolve()}")


if __name__ == "__main__":
    main()
