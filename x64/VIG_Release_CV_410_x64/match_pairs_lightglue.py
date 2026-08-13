#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Deterministisches Image-Pair-Matching mit SuperPoint + LightGlue.

Ziel: reproduzierbare Ergebnisse (gleiche Anzahl Keypoints, gleiche Matches)
bei mehrfachem Aufruf, ohne die GPU komplett auszubremsen.

Wichtige Punkte im Vergleich zu vorherigen Versionen:
- warn_only=True bei use_deterministic_algorithms: wirft KEINEN Fehler bei
  Operationen ohne deterministische CUDA-Implementierung, sondern fällt
  automatisch zurück / warnt im Log. Dadurch bleibt die GPU nutzbar.
- TF32 bleibt standardmäßig AN (kannst du unten deaktivieren, falls du nach
  einem Testlauf siehst, dass es die Ursache für Abweichungen ist).
- set_num_threads(1) wurde entfernt (betrifft nur CPU-Threading, nicht die
  eigentliche Determinismus-Frage, bremst aber unnötig Vor-/Nachverarbeitung).
- Modelle UND Bilder werden explizit auf `device` verschoben (im alten
  Skript fehlte das .to(device) für die Modelle).
"""

import os

# -----------------------------------------------------------------------------
# Environment (MUSS vor dem Torch-Import gesetzt werden)
# -----------------------------------------------------------------------------

os.environ["PYTHONHASHSEED"] = "42"
os.environ["CUBLAS_WORKSPACE_CONFIG"] = ":4096:8"

import argparse
import random
import warnings
from pathlib import Path

import numpy as np
import torch

from lightglue import LightGlue, SuperPoint, DISK, ALIKED, SIFT
from lightglue.utils import load_image, rbd

os.environ["TORCH_HOME"] = os.path.join(os.path.dirname(__file__), "models")

# -----------------------------------------------------------------------------
# Globale Konfiguration
# -----------------------------------------------------------------------------

SEED = 42

# Falls du nach einem Testlauf siehst, dass TF32 zu Abweichungen führt
# (typischerweise nur auf Ampere+ GPUs relevant), setze das hier auf True.
DISABLE_TF32 = False

import hashlib

def file_hash(path):
    with open(path, "rb") as f:
        return hashlib.md5(f.read()).hexdigest()
    

def set_determinism(seed: int = 42, disable_tf32: bool = False) -> None:
    random.seed(seed)
    np.random.seed(seed)

    torch.manual_seed(seed)

    if torch.cuda.is_available():
        torch.cuda.manual_seed(seed)
        torch.cuda.manual_seed_all(seed)

    # cuDNN: feste Kernel-Auswahl statt Auto-Tuning-Benchmark
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True

    if disable_tf32:
        torch.backends.cuda.matmul.allow_tf32 = False
        torch.backends.cudnn.allow_tf32 = False

    # warn_only=True: bricht NICHT ab, wenn eine Operation (z.B. in
    # SuperPoint/LightGlue) keine deterministische CUDA-Implementierung hat.
    # Stattdessen gibt's eine UserWarning im Log -> siehst du, was ggf.
    # noch nicht 100% deterministisch ist, ohne dass das Skript crasht.
    torch.use_deterministic_algorithms(True, warn_only=True)


def synchronize(device: torch.device) -> None:
    if device.type == "cuda":
        torch.cuda.synchronize()


# -----------------------------------------------------------------------------

def main() -> None:

    # Alle Nicht-Deterministik-Warnungen sichtbar machen (statt nur beim
    # ersten Auftreten)
    warnings.filterwarnings("always", category=UserWarning)

    set_determinism(SEED, disable_tf32=DISABLE_TF32)

    parser = argparse.ArgumentParser(
        description="Deterministisches Image-Pair-Matching mit LightGlue",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--left_image", required=True, help="Pfad zum linken Bild")
    parser.add_argument("--right_image", required=True, help="Pfad zum rechten Bild")
    parser.add_argument("--output_dir", required=True, help="Pfad zum Output-Verzeichnis")
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device : {device}")

    # Quick test
    print(f"Hash left : {file_hash(args.left_image)}")
    print(f"Hash right: {file_hash(args.right_image)}")

    # -------------------------------------------------------------------------
    # Modelle (explizit auf device)
    # -------------------------------------------------------------------------

    extractor = SuperPoint(max_num_keypoints=None).eval().to(device)

    matcher = LightGlue(
        features="superpoint",
        depth_confidence=-1,
        width_confidence=-1,
        filter_threshold=0.15,  # robuster als 0.09, ggf. weniger Matches
        flash=False,
        mp=False,
    ).eval().to(device)

    # -------------------------------------------------------------------------
    # Bilder
    # -------------------------------------------------------------------------

    image0 = load_image(args.left_image).to(device)
    image1 = load_image(args.right_image).to(device)

    synchronize(device)

    # -------------------------------------------------------------------------
    # Inferenz
    # -------------------------------------------------------------------------

    with torch.inference_mode():
        feats0 = extractor.extract(image0)
        synchronize(device)

        feats1 = extractor.extract(image1)
        synchronize(device)

        matches01 = matcher({"image0": feats0, "image1": feats1})
        synchronize(device)

    feats0, feats1, matches01 = map(rbd, (feats0, feats1, matches01))

    kpts0 = feats0["keypoints"]
    kpts1 = feats1["keypoints"]
    matches = matches01["matches"]

    m_kpts0 = kpts0[matches[:, 0]]
    m_kpts1 = kpts1[matches[:, 1]]

    print(f"Detected keypoints: image0={len(kpts0)}, image1={len(kpts1)}")
    print(f"Matches: {len(matches)}")


    # test
    scores0 = matches01["matching_scores0"]  # pro Keypoint in image0, auch für Nicht-Matches
    scores0_np = scores0.detach().cpu().numpy()

    print(f"Score-Stats image0: min={scores0_np.min():.4f}, "
        f"max={scores0_np.max():.4f}, mean={scores0_np.mean():.4f}")

    for t in [0.15, 0.10, 0.05, 0.02, 0.0]:
        print(f"  Kandidaten über Threshold {t}: {(scores0_np > t).sum()}")


    d1 = feats1["descriptors"].squeeze(0)  # [N, D]

    # Wie stark streuen die Deskriptoren überhaupt? Sehr niedrige Werte
    # deuten auf Kollaps/Redundanz hin.
    print(f"Descriptor std (mean über Dims): {d1.std(dim=0).mean().item():.6f}")

    # Paarweise Cosine-Similarity auf einer Stichprobe (bei 5458 Punkten
    # nicht alle-gegen-alle rechnen)
    sample = d1[:500]
    sample_n = torch.nn.functional.normalize(sample, dim=1)
    sim = sample_n @ sample_n.T
    sim.fill_diagonal_(0)
    print(f"Mean pairwise cosine sim (Stichprobe): {sim.mean().item():.4f}")
    print(f"Max pairwise cosine sim (Stichprobe, off-diag): {sim.max().item():.4f}")

    # Zur Kontrolle: NaN/Inf trotzdem ausschließen
    print(f"NaN: {torch.isnan(d1).any().item()}, Inf: {torch.isinf(d1).any().item()}")



    # 1) Lokale statt globale Ähnlichkeit: Nächste-Nachbarn im Bildraum,
    #    nicht zufällige Paare
    kpts1_np = kpts1.detach().cpu().numpy()
    d1 = feats1["descriptors"].squeeze(0)
    d1n = torch.nn.functional.normalize(d1, dim=1)

    from scipy.spatial import cKDTree
    tree = cKDTree(kpts1_np)
    # für eine Stichprobe: 5 nächste räumliche Nachbarn suchen, deren
    # Deskriptor-Ähnlichkeit checken
    sample_idx = np.random.choice(len(kpts1_np), 200, replace=False)
    local_sims = []
    for i in sample_idx:
        dists, idxs = tree.query(kpts1_np[i], k=6)  # sich selbst + 5 Nachbarn
        for j in idxs[1:]:
            sim = (d1n[i] @ d1n[j]).item()
            local_sims.append(sim)

    local_sims = np.array(local_sims)
    print(f"Lokale NN-Similarity: mean={local_sims.mean():.4f}, "
        f"max={local_sims.max():.4f}, >0.9: {(local_sims>0.9).sum()}/{len(local_sims)}")

    # 2) Ist es "global flach" oder "relativ noch ein Peak pro Zeile,
    #    nur insgesamt niedriger skaliert"?
    scores_matrix = matches01.get("scores")  # falls vorhanden, sonst matching_scores0 row-wise
    row_max = scores0_np  # ihr habt ja schon row-wise max effektiv in matching_scores0
    print(f"Anteil Zeilen mit row_max > 3x row_mean: "
        f"{(scores0_np > 3*scores0_np.mean()).sum()} von {len(scores0_np)}")


    print(f"image0 shape: {tuple(image0.shape)}")  # [C,H,W]
    print(f"image1 shape: {tuple(image1.shape)}")

    # Falls vorhanden: welche Bildgröße hat LightGlue für die Normierung
    # tatsächlich verwendet?
    print(f"feats0 image_size: {feats0.get('image_size')}")
    print(f"feats1 image_size: {feats1.get('image_size')}")

    # matches01 (vor rbd) enthält meist "log_assignment" oder die Scores
    # inkl. Dustbin-Spalte/Zeile - je nach LightGlue-Version anders benannt
    print(matches01.keys())
    # test end

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


if __name__ == "__main__":
    main()


# from pathlib import Path
# from lightglue import LightGlue, SuperPoint, DISK, ALIKED, SIFT
# from lightglue.utils import load_image, rbd
# import torch
# import argparse
# import os
# from pathlib import Path

# os.environ["TORCH_HOME"] = os.path.join(os.path.dirname(__file__), "models")

# if __name__ == '__main__':
#     parser = argparse.ArgumentParser(
#         description='Image pair matching with LightGlue',
#         formatter_class=argparse.ArgumentDefaultsHelpFormatter)

#     parser.add_argument(
#         '--left_image', type=str, default='',
#         help='Path to the true image')
#     parser.add_argument(
#         '--right_image', type=str, default='',
#         help='Path to the synth image')
#     parser.add_argument(
#         '--output_dir', type=str, default='',
#         help='Path to the output dir')
    
#     opt = parser.parse_args()

#     left_image = Path(opt.left_image)
#     print ("Left image", left_image)
#     right_image = Path(opt.right_image)
#     print ("Right image", right_image)
#     output_dir = Path(opt.output_dir)
#     print ("output dir", output_dir)


#     output_dir.mkdir(exist_ok=True, parents=True)
#     print('Will write matches to directory \"{}\"'.format(output_dir))
    
#     torch.set_grad_enabled(False)
#     device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # 'mps', 'cpu'
    
#     #The default values give a good trade-off between speed and accuracy. To maximize the accuracy, use all keypoints and disable the adaptive mechanisms:
#     extractor = SuperPoint(max_num_keypoints=None)
#     matcher = LightGlue(features='superpoint', depth_confidence=-1, width_confidence=-1, filter_threshold = 0.09)

#     image0 = load_image(left_image)
#     image1 = load_image(right_image)

#     feats0 = extractor.extract(image0.to(device))
#     feats1 = extractor.extract(image1.to(device))
#     matches01 = matcher({"image0": feats0, "image1": feats1})
#     feats0, feats1, matches01 = [
#         rbd(x) for x in [feats0, feats1, matches01]
#     ]  # remove batch dimension

#     kpts0, kpts1, matches = feats0["keypoints"], feats1["keypoints"], matches01["matches"]
#     m_kpts0, m_kpts1 = kpts0[matches[..., 0]], kpts1[matches[..., 1]]

#     print ("number of matches", len(m_kpts0), len(m_kpts1))

#     # Keep the matching keypoints.
#     # Save Keypoints (inliers) to disk
# 	# generate path
#     path_output =  output_dir / 'kpts.txt'
#     print(path_output)

#     print("lists have same length") if len(m_kpts0) == len(m_kpts1) else print ("ohoh, lists have not the same length")       
#     f = open(path_output, "w")
#     for i in range(len(m_kpts0)):

#         p = str(i) + " " + str(m_kpts0[i][0].item()) + " " + str(m_kpts0[i][1].item()) + " " + str(m_kpts1[i][0].item()) + " " + str(m_kpts1[i][1].item()) + "\n"
#         #print (p)
#         f.write(p)
#     f.close()
