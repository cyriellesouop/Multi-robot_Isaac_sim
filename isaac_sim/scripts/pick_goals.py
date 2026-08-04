#!/usr/bin/env python3
"""
pick_goals.py

Interactive tool for picking navigation goal coordinates directly off the
occupancy map PNG, converting pixel clicks into world (x, y) coordinates
using the map YAML's resolution and origin — the same convention Nav2's
map_server uses.

Usage:
    python3 pick_goals.py --map carter_warehouse_navigation.png \
                           --resolution 0.05 \
                           --origin -11.975 -17.975

Click anywhere on the displayed map window. For each click, the terminal
prints:
    - pixel coordinates (row, col)
    - world coordinates (x, y)  <- use these in goals.txt
    - occupancy status (free / occupied / unknown), based on pixel intensity

Press 'q' or Esc to quit.

Coordinate convention (matches ROS map_server / nav2_map_server):
    - The YAML's "origin" is the world (x, y) of the BOTTOM-LEFT pixel
      of the image.
    - Image row 0 is the TOP of the image (standard image convention),
      so row increases downward while world y increases upward — the
      row axis must be flipped when converting to world coordinates.

    world_x = origin_x + col * resolution
    world_y = origin_y + (image_height - 1 - row) * resolution

Occupancy convention (matches map_server's trinary interpretation):
    - Pixel intensity near 255 (white)  -> free
    - Pixel intensity near 0   (black)  -> occupied
    - Pixel intensity around ~205 (gray) -> unknown
  (Exact thresholds configurable via --free-thresh / --occupied-thresh,
   matching the same fields used in your map YAML.)
"""

import argparse
import cv2
import numpy as np


def pixel_to_world(row: int, col: int, img_height: int, resolution: float, origin_x: float, origin_y: float):
    world_x = origin_x + col * resolution
    world_y = origin_y + (img_height - 1 - row) * resolution
    return world_x, world_y


def classify_occupancy(pixel_value: int, free_thresh: float, occupied_thresh: float):
    # map_server interprets pixel intensity as a probability of occupancy
    # after normalizing to [0, 1]; thresholds from the YAML apply directly.
    normalized = pixel_value / 255.0
    # Note: map_server's actual formula also depends on "negate"; this
    # simplified version assumes negate=0 (standard: white=free, black=occupied),
    # which matches your carter_warehouse_navigation.yaml.
    occ_prob = 1.0 - normalized
    if occ_prob <= free_thresh:
        return "FREE"
    elif occ_prob >= occupied_thresh:
        return "OCCUPIED"
    else:
        return "UNKNOWN"


def check_clearance(img, row, col, resolution, clearance_m, free_thresh, occupied_thresh):
    """
    Checks every pixel within `clearance_m` meters of (row, col), not just
    the clicked pixel itself. A single free pixel next to a wall/obstacle
    (e.g. a forklift) is not enough — the robot has a physical footprint,
    and Nav2's costmap inflates obstacles outward, so a goal needs a real
    clearance margin, not just a single non-occupied point.

    Returns (is_clear: bool, worst_status: str, min_clearance_found_m: float)
    """
    radius_px = int(round(clearance_m / resolution))
    img_height, img_width = img.shape

    worst_status = "FREE"
    found_any_non_free = False

    for dr in range(-radius_px, radius_px + 1):
        for dc in range(-radius_px, radius_px + 1):
            if dr * dr + dc * dc > radius_px * radius_px:
                continue  # only check within the circular radius, not a square
            r, c = row + dr, col + dc
            if not (0 <= r < img_height and 0 <= c < img_width):
                # Treat out-of-bounds (near map edge) as unsafe, conservatively
                return False, "OUT_OF_BOUNDS", 0.0
            status = classify_occupancy(int(img[r, c]), free_thresh, occupied_thresh)
            if status != "FREE":
                found_any_non_free = True
                worst_status = status if status == "OCCUPIED" else worst_status

    is_clear = not found_any_non_free
    return is_clear, worst_status, clearance_m


def main():
    parser = argparse.ArgumentParser(description="Click the occupancy map to get world coordinates.")
    parser.add_argument("--map", required=True, help="Path to the occupancy map PNG.")
    parser.add_argument("--resolution", type=float, required=True, help="Meters per pixel (from YAML).")
    parser.add_argument("--origin", type=float, nargs=2, required=True,
                         metavar=("ORIGIN_X", "ORIGIN_Y"),
                         help="World (x, y) of the bottom-left pixel (from YAML 'origin' field).")
    parser.add_argument("--free-thresh", type=float, default=0.196, help="From YAML free_thresh (default: 0.196).")
    parser.add_argument("--occupied-thresh", type=float, default=0.65, help="From YAML occupied_thresh (default: 0.65).")
    parser.add_argument("--clearance", type=float, default=0.3,
                         help="Required clear radius in meters around each clicked point "
                              "(default: 0.3m — adjust to match your robot's footprint radius "
                              "plus a safety margin; Nova Carter's footprint is roughly 0.4m wide, "
                              "so 0.3-0.4m clearance is a reasonable starting point).")
    args = parser.parse_args()

    img = cv2.imread(args.map, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Could not load image: {args.map}")

    img_height, img_width = img.shape
    print(f"Loaded map: {args.map} ({img_width} x {img_height} px)")
    print(f"World size: {img_width * args.resolution:.2f} m x {img_height * args.resolution:.2f} m")
    print(f"Origin (world coords of bottom-left pixel): {args.origin}")
    print("Click on the map window to get world coordinates. Press 'q' or Esc to quit.\n")

    display_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    picked_points = []

    def on_click(event, col, row, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            world_x, world_y = pixel_to_world(
                row, col, img_height, args.resolution, args.origin[0], args.origin[1]
            )
            pixel_value = int(img[row, col])
            status = classify_occupancy(pixel_value, args.free_thresh, args.occupied_thresh)
            is_clear, worst_status, checked_radius = check_clearance(
                img, row, col, args.resolution, args.clearance, args.free_thresh, args.occupied_thresh
            )
            picked_points.append((world_x, world_y, status, is_clear))

            print(f"pixel=({row},{col})  world=({world_x:.3f}, {world_y:.3f})  "
                  f"pixel_val={pixel_value}  status={status}  "
                  f"clearance_ok({args.clearance}m)={is_clear}")
            if status != "FREE":
                print("  ^ WARNING: not clearly free space, pick elsewhere if possible.")
            elif not is_clear:
                print(f"  ^ WARNING: point itself is free, but within {args.clearance}m there is "
                      f"{worst_status} space (e.g. a nearby wall/obstacle) — not enough clearance "
                      f"for the robot's footprint. Pick a point further from obstacles.")

            # Green = genuinely clear (point + surrounding clearance radius both OK)
            # Orange = free pixel but insufficient clearance nearby
            # Red = occupied/unknown at the point itself
            if status != "FREE":
                color = (0, 0, 255)      # red
            elif not is_clear:
                color = (0, 165, 255)    # orange
            else:
                color = (0, 255, 0)      # green
            cv2.circle(display_img, (col, row), 4, color, -1)
            cv2.imshow("Occupancy Map (click to pick goals)", display_img)

    cv2.namedWindow("Occupancy Map (click to pick goals)", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Occupancy Map (click to pick goals)", on_click)
    cv2.imshow("Occupancy Map (click to pick goals)", display_img)

    while True:
        key = cv2.waitKey(20) & 0xFF
        if key == ord('q') or key == 27:  # 'q' or Esc
            break

    cv2.destroyAllWindows()

    if picked_points:
        print("\n--- Summary of picked points (world coordinates) ---")
        for i, (x, y, status, is_clear) in enumerate(picked_points, start=1):
            clear_note = "OK" if (status == "FREE" and is_clear) else "INSUFFICIENT CLEARANCE / NOT FREE"
            print(f"{i}. x={x:.3f}  y={y:.3f}  ({status}, {clear_note})")
        print("\nFor goals.txt, format each line as:")
        print("  pose.x pose.y 0 0 0 1   (identity orientation, unit quaternion)")
        print("\nOnly use points marked 'OK' — others risk the recovery-behavior")
        print("collision loop seen when a goal sits too close to an obstacle")
        print("(e.g. a forklift) despite the exact pixel being free.")


if __name__ == "__main__":
    main()
