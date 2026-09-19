import argparse
from pathlib import Path
from tqdm import tqdm
from hmr4d.utils.pylogger import Log
import subprocess
import os


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--folder", type=str)
    parser.add_argument("-d", "--output_root", type=str, default=None)
    parser.add_argument("-s", "--static_cam", action="store_true", help="If true, skip DPVO")
    parser.add_argument(
        "--skip_render",
        action="store_true",
        help="Save hmr4d_results.pt without PyTorch3D CUDA rendering.",
    )
    args = parser.parse_args()

    folder = Path(args.folder)
    output_root = args.output_root

    # Run demo.py for each .mp4 file
    mp4_paths = sorted(list(folder.glob("*.mp4")) + list(folder.glob("*.MP4")))
    Log.info(f"Found {len(mp4_paths)} .mp4 files in {folder}")
    for mp4_path in tqdm(mp4_paths):
        command = ["python", "tools/demo/demo.py", "--video", str(mp4_path)]
        if output_root is not None:
            command += ["--output_root", output_root]
        if args.static_cam:
            command += ["-s"]
        if args.skip_render:
            command += ["--skip_render"]
        Log.info(f"Running: {' '.join(command)}")
        env = dict(os.environ)
        env.setdefault("TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD", "1")
        subprocess.run(command, env=env, check=True)
