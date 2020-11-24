# -*- coding: utf-8 -*-
"""
Spyder Editor

Dies ist eine temporäre Skriptdatei.
"""


import cv2

import matplotlib.pyplot as plt

import numpy as np

import os

from PIL import Image

from skimage.feature import match_descriptors
from skimage.measure import ransac
from skimage.transform import ProjectiveTransform


# Change the pair index here (possible values: 1, 2 or 3)
pair_idx = 5
assert(pair_idx in [1, 2, 3, 4, 5])

# Loading the features
pair_path = os.path.join('images', 'pair_%d' % pair_idx)
print(pair_path)

image1 = np.array(Image.open(os.path.join(pair_path, '1.jpg')))
image2 = np.array(Image.open(os.path.join(pair_path, '2.jpg')))
feat1 = np.load(os.path.join(pair_path, '1.jpg.d2-net'))
feat2 = np.load(os.path.join(pair_path, '2.jpg.d2-net'))

# Mutual nearest neighbors matching
matches = match_descriptors(feat1['descriptors'], feat2['descriptors'], cross_check=True)
print('Number of raw matches: %d.' % matches.shape[0])

# Homography fitting
keypoints_left = feat1['keypoints'][matches[:, 0], : 2]
keypoints_right = feat2['keypoints'][matches[:, 1], : 2]
np.random.seed(0)
model, inliers = ransac(
    (keypoints_left, keypoints_right),
    ProjectiveTransform, min_samples=4,
    residual_threshold=4, max_trials=10000
)
n_inliers = np.sum(inliers)
print('Number of inliers: %d.' % n_inliers)

# Plotting
inlier_keypoints_left = [cv2.KeyPoint(point[0], point[1], 1) for point in keypoints_left[inliers]]
inlier_keypoints_right = [cv2.KeyPoint(point[0], point[1], 1) for point in keypoints_right[inliers]]
placeholder_matches = [cv2.DMatch(idx, idx, 1) for idx in range(n_inliers)]
image3 = cv2.drawMatches(image1, inlier_keypoints_left, image2, inlier_keypoints_right, placeholder_matches, None)
plt.figure(figsize=(15,15))
plt.imshow(image3)
plt.axis('off')
plt.show()


# Save Keypoints (inliers) to disk
# generate path
path_output_1 = os.path.join(pair_path, 'kpts.txt')
print(path_output_1)

f = open(path_output_1, "w")
for i, kpts_left in enumerate(inlier_keypoints_left):
    kpts_right = inlier_keypoints_right[i]
    p = str(i) + "," + str(kpts_left.pt[0]) + "," + str(kpts_left.pt[1]) + "," + str(kpts_right.pt[0]) + "," + str(kpts_right.pt[1]) + "\n"
    f.write(p)
f.close()

