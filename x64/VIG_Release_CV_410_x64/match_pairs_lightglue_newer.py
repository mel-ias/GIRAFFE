#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

# -----------------------------------------------------------------------------
# Environment (MUST be before importing torch)
# -----------------------------------------------------------------------------

os.environ["PYTHONHASHSEED"] = "42"
os.environ["CUBLAS_WORKSPACE_CONFIG"] = ":4096:8"


# Optional:
# os.environ["CUDA_LAUNCH_BLOCKING"] = "1"

import argparse
import random
from pathlib import Path

import numpy as np
import torch

from lightglue import LightGlue, SuperPoint
from lightglue.utils import load_image, rbd

os.environ["TORCH_HOME"] = os.path.join(os.path.dirname(__file__), "models")

# -----------------------------------------------------------------------------
# Global configuration
# -----------------------------------------------------------------------------

SEED = 42


def set_determinism(seed=42):

    random.seed(seed)
    np.random.seed(seed)

    torch.manual_seed(seed)

    if torch.cuda.is_available():
        torch.cuda.manual_seed(seed)
        torch.cuda.manual_seed_all(seed)

    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True

    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False

    torch.use_deterministic_algorithms(True)

    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)


def synchronize(device):
    if device.type == "cuda":
        torch.cuda.synchronize()



# -----------------------------------------------------------------------------

def main():

    set_determinism(SEED)

    parser = argparse.ArgumentParser()

    parser.add_argument("--left_image", required=True)
    parser.add_argument("--right_image", required=True)
    parser.add_argument("--output_dir", required=True)

    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    device = torch.device(
        "cuda" if torch.cuda.is_available() else "cpu"
    )

    print(f"Device : {device}")

    # -------------------------------------------------------------------------
    # Models
    # -------------------------------------------------------------------------

    extractor = (
        SuperPoint(
            max_num_keypoints=None
        )
        .eval()
        .to(device)
    )

    matcher = (
        LightGlue(
            features="superpoint",

            depth_confidence=-1,
            width_confidence=-1,

            filter_threshold=0.15, #robuster als 0.09, aber ggf. weniger matches

            flash=False,
            mp=False,
        )
        .eval()
        .to(device)
    )

    # -------------------------------------------------------------------------
    # Images
    # -------------------------------------------------------------------------

    image0 = load_image(args.left_image).to(device)
    image1 = load_image(args.right_image).to(device)

    synchronize(device)

    # -------------------------------------------------------------------------
    # Inference
    # -------------------------------------------------------------------------

    with torch.inference_mode():

        feats0 = extractor.extract(image0)

        synchronize(device)

        feats1 = extractor.extract(image1)

        synchronize(device)


        matches01 = matcher(
            {
                "image0": feats0,
                "image1": feats1,
            }
        )

        synchronize(device)

    feats0, feats1, matches01 = map(
        rbd,
        (feats0, feats1, matches01),
    )

    kpts0 = feats0["keypoints"]
    kpts1 = feats1["keypoints"]

    matches = matches01["matches"]

    m_kpts0 = kpts0[matches[:, 0]]
    m_kpts1 = kpts1[matches[:, 1]]

    print(f"Matches: {len(matches)}")

    output_file = output_dir / "kpts.txt"

    with open(output_file, "w") as f:

        for i, (p0, p1) in enumerate(zip(m_kpts0, m_kpts1)):

            f.write(
                f"{i} "
                f"{p0[0].item()} "
                f"{p0[1].item()} "
                f"{p1[0].item()} "
                f"{p1[1].item()}\n"
            )

    print("Done.")


# -----------------------------------------------------------------------------

if __name__ == "__main__":
    main()