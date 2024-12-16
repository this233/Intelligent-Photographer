import sys
import os
import argparse
import numpy as np
import cv2
from PIL import Image
import torch
import torch.nn.functional as F
from torch.autograd import Variable
import torchvision
from torchvision import transforms
import dlib
import hopenet, utils

def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(description='Head pose estimation using the Hopenet network.')
    parser.add_argument('--gpu', dest='gpu_id', help='GPU device id to use [0]', default=0, type=int)
    parser.add_argument('--snapshot', dest='snapshot', help='Path of model snapshot.', required=True, type=str)
    parser.add_argument('--face_model', dest='face_model', help='Path of DLIB face detection model.', required=True, type=str)
    parser.add_argument('--image', dest='image_path', help='Path of input image.', required=True, type=str)
    parser.add_argument('--output', dest='output_path', help='Path to save output image.', default='output.jpg', type=str)
    args = parser.parse_args()
    return args

def main():
    args = parse_args()

    # Load Hopenet model
    model = hopenet.Hopenet(torchvision.models.resnet.Bottleneck, [3, 4, 6, 3], 66)
    saved_state_dict = torch.load(args.snapshot, map_location=torch.device('cuda' if torch.cuda.is_available() else 'cpu'))
    model.load_state_dict(saved_state_dict)
    model.eval()

    # Use GPU if available
    device = torch.device(f"cuda:{args.gpu_id}" if torch.cuda.is_available() else "cpu")
    model = model.to(device)

    # Load face detection model
    cnn_face_detector = dlib.cnn_face_detection_model_v1(args.face_model)

    # Load image
    if not os.path.exists(args.image_path):
        sys.exit('Image does not exist')

    image = cv2.imread(args.image_path)
    if image is None:
        sys.exit('Failed to load image')

    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Detect faces
    dets = cnn_face_detector(image_rgb, 1)

    transformations = transforms.Compose([
        transforms.Resize(224),
        transforms.CenterCrop(224),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])

    idx_tensor = torch.FloatTensor([idx for idx in range(66)]).to(device)

    max_conf = 0
    index = -1
    for i, det in enumerate(dets):
        if det.confidence > max_conf:
            max_conf = det.confidence
            index = i

    for i,det in enumerate(dets):
        if i != index:
            continue
        x_min, y_min, x_max, y_max = det.rect.left(), det.rect.top(), det.rect.right(), det.rect.bottom()
        
        conf = det.confidence
        print(i,x_min,x_max,y_min,y_max,conf)
        cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            
            
        # if conf > 0.1:
        bbox_width = abs(x_max - x_min)
        bbox_height = abs(y_max - y_min)

        
        x_min = max(0, x_min - int(0.2 * bbox_width))
        x_max = min(image.shape[1], x_max + int(0.2 * bbox_width))
        y_min = max(0, y_min - int(0.3 * bbox_height))
        y_max = min(image.shape[0], y_max + int(0.1 * bbox_height))

        # Crop and preprocess the face image
        face_img = image_rgb[y_min:y_max, x_min:x_max]
        face_img = Image.fromarray(face_img)
        face_img = transformations(face_img).unsqueeze(0).to(device)

        # Head pose estimation
        yaw, pitch, roll = model(face_img)

        yaw_predicted = torch.sum(F.softmax(yaw, dim=1)[0] * idx_tensor) * 3 - 99
        pitch_predicted = torch.sum(F.softmax(pitch, dim=1)[0] * idx_tensor) * 3 - 99
        roll_predicted = torch.sum(F.softmax(roll, dim=1)[0] * idx_tensor) * 3 - 99

        # Draw axis on the image
        utils.draw_axis(image, yaw_predicted.item(), pitch_predicted.item(), roll_predicted.item(),
                        tdx=(x_min + x_max) // 2, tdy=(y_min + y_max) // 2, size=bbox_height // 2)
        # 画框框
        
        print(i, x_min, x_max, y_min, y_max, bbox_height,bbox_width, yaw_predicted.item(), pitch_predicted.item(), roll_predicted.item())
        
        # print(i, x_min, x_max, y_min, y_max, bbox_height,bbox_width, yaw_predicted.item(), pitch_predicted.item(), roll_predicted.item())
        center_x = (x_min + x_max) / 2
        center_y = (y_min + y_max) / 2
        # 输出中心点位置在图像位置的比例
        center_x_ratio = center_x / image.shape[1] # width
        center_y_ratio = center_y / image.shape[0] # height
        center_y_ratio = 1-center_y_ratio
        
        print("x: ", center_x_ratio, "y: ", center_y_ratio, "偏航角：", yaw_predicted.item(), "俯仰角：", pitch_predicted.item(), "翻滚角：", roll_predicted.item())
    # Save output image
    cv2.imwrite(args.output_path, image)
    print(f"Output saved to {args.output_path}")

if __name__ == '__main__':
    main()
