import torch
import torchvision
from torchvision.models.detection.mask_rcnn import MaskRCNNPredictor
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor

from torchvision import transforms
import argparse

from PIL import Image
import numpy as np

from utils.visualize import visualize, save_image
from tqdm import tqdm
import os
import cv2

convert_tensor = transforms.ToTensor()

def post_process(detections, num_detections, c_thresh = 0.75):
    print(detections)
    p_detections = []
    for i in range(num_detections):
        if detections['scores'][i] > c_thresh:
            detection = {"bbox": detections['boxes'].detach().numpy()[i].reshape(4,),
                         "mask": detections['masks'].detach().numpy()[i].reshape(240, 320),
                         "label": detections['labels'].detach().numpy()[i]}
            p_detections.append(detection)
    return p_detections

def get_trained_model(weights_path, num_classes = 5):
    # load an instance segmentation model pre-trained on COCO
    model = torchvision.models.detection.maskrcnn_resnet50_fpn()
    # replace the pre-trained head with a new one
    # get number of input features for the classifier
    in_features = model.roi_heads.box_predictor.cls_score.in_features
    model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)

    # now get the number of input features for the mask classifier
    in_features_mask = model.roi_heads.mask_predictor.conv5_mask.in_channels
    hidden_layer = 256
    # and replace the mask predictor with a new one
    model.roi_heads.mask_predictor = MaskRCNNPredictor(in_features_mask,
                                                       hidden_layer,
                                                       num_classes)


    model.load_state_dict(torch.load(weights_path))

    return model


def evaluate():
    parser = argparse.ArgumentParser(description="Evaluate validation data.")
    parser.add_argument("-i", "--img_directory", type=str, default="data_test/images/", help="Path to image to evaluate on")
    parser.add_argument("-w", "--weights", type=str, default="/home/anaveen/Documents/nasa_ws/astrobee-detection-pipeline/src/handrail_segmentation/pytorch_mrcnn/checkpoints/mrcnn_ckpt_0.pth")
    parser.add_argument("-n", "--nms_thesh", type=float, default=0.7)
    args = parser.parse_args()

    model = get_trained_model(args.weights)
    model.eval()

    device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')

    img_paths = [os.path.join(args.img_directory, img_name) for img_name in list(sorted(os.listdir(args.img_directory)))]

    for img_path in tqdm(img_paths):
        img = Image.open(img_path).convert("RGB")
        img = [convert_tensor(img)]
        torch.cuda.synchronize()

        detections = model(img)[0]
        n = len(detections['labels'])
        detections = post_process(detections, n)

        annotated_img = cv2.imread(img_path, cv2.IMREAD_COLOR)

        for detection in detections:
            bbox, mask, label = detection.values()

            np.place(mask, mask > args.nms_thesh, label)
            np.place(mask, mask <= args.nms_thesh, 0)

            annotated_img = visualize(annotated_img, bbox, mask, label)

        save_image(annotated_img, img_path)

if __name__=='__main__':
    evaluate()
