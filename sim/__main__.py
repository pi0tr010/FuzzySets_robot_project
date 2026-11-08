import argparse
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def parse_size(text: str):
    w, _, h = text.lower().partition("x")
    return int(w), int(h)


def main():
    ap = argparse.ArgumentParser(prog="python -m sim", description="Maze robot simulator")
    ap.add_argument("--map", type=Path, default=ROOT / "maps" / "demo.json",
                    help="map file to load and save (Ctrl+S)")
    ap.add_argument("--new", metavar="WxH", type=parse_size,
                    help="create an empty map of WxH cells (saved to --map on Ctrl+S)")
    ap.add_argument("--cell", type=float, default=0.10, help="cell size in meters for --new")
    ap.add_argument("--robot", type=Path, default=ROOT / "firmware" / "build" / "robot_sim",
                    help="robot program built with the simulator HAL")
    ap.add_argument("--speed", type=float, default=1.0, help="initial time scale")
    ap.add_argument("--seed", type=int, default=1, help="sensor noise seed (same seed, same run)")
    ap.add_argument("--headless", action="store_true", help="no window (for tests)")
    ap.add_argument("--duration", type=float, help="stop after this many simulated seconds")
    ap.add_argument("--screenshot", type=Path, help="save the last frame as PNG")
    args = ap.parse_args()

    if args.headless:
        os.environ["SDL_VIDEODRIVER"] = "dummy"
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

    from .app import App
    from .world import World

    if args.new:
        world = World(args.new[0], args.new[1], args.cell)
    elif args.map.exists():
        world = World.load(args.map)
    else:
        sys.exit(f"map {args.map} not found (use --new WxH to create one)")

    if not args.robot.exists():
        sys.exit(f"robot program {args.robot} not found, run: make -C firmware sim")

    App(world, args.map, [str(args.robot)], ROOT / "firmware",
        speed=args.speed, seed=args.seed, headless=args.headless).run(
        duration_s=args.duration, screenshot=args.screenshot)


if __name__ == "__main__":
    main()
