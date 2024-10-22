
from pathlib import Path
from lightglue import LightGlue, SuperPoint, DISK, ALIKED, SIFT
from lightglue.utils import load_image, rbd
import torch
import argparse


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Image pair matching with LightGlue',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument(
        '--left_image', type=str, default='',
        help='Path to the true image')
    parser.add_argument(
        '--right_image', type=str, default='',
        help='Path to the synth image')
    parser.add_argument(
        '--output_dir', type=str, default='',
        help='Path to the output dir')
    
    opt = parser.parse_args()

    left_image = Path(opt.left_image)
    print ("Left image", left_image)
    right_image = Path(opt.right_image)
    print ("Right image", right_image)
    output_dir = Path(opt.output_dir)
    print ("output dir", output_dir)


    output_dir.mkdir(exist_ok=True, parents=True)
    print('Will write matches to directory \"{}\"'.format(output_dir))
    
    torch.set_grad_enabled(False)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # 'mps', 'cpu'
    
    #The default values give a good trade-off between speed and accuracy. To maximize the accuracy, use all keypoints and disable the adaptive mechanisms:
    extractor = SuperPoint(max_num_keypoints=None)
    matcher = LightGlue(features='superpoint', depth_confidence=-1, width_confidence=-1, filter_threshold = 0.09)

    image0 = load_image(left_image)
    image1 = load_image(right_image)

    feats0 = extractor.extract(image0.to(device))
    feats1 = extractor.extract(image1.to(device))
    matches01 = matcher({"image0": feats0, "image1": feats1})
    feats0, feats1, matches01 = [
        rbd(x) for x in [feats0, feats1, matches01]
    ]  # remove batch dimension

    kpts0, kpts1, matches = feats0["keypoints"], feats1["keypoints"], matches01["matches"]
    m_kpts0, m_kpts1 = kpts0[matches[..., 0]], kpts1[matches[..., 1]]

    print ("number of matches", len(m_kpts0), len(m_kpts1))

    # Keep the matching keypoints.
    # Save Keypoints (inliers) to disk
	# generate path
    path_output =  output_dir / 'kpts.txt'
    print(path_output)

    print("lists have same length") if len(m_kpts0) == len(m_kpts1) else print ("ohoh, lists have not the same length")       
    f = open(path_output, "w")
    for i in range(len(m_kpts0)):

        p = str(i) + " " + str(m_kpts0[i][0].item()) + " " + str(m_kpts0[i][1].item()) + " " + str(m_kpts1[i][0].item()) + " " + str(m_kpts1[i][1].item()) + "\n"
        #print (p)
        f.write(p)
    f.close()
