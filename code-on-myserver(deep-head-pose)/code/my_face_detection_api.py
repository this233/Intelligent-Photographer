from fastapi import FastAPI, File, UploadFile, HTTPException
from pydantic import BaseModel
import cv2
import numpy as np
import torch
import torch.nn.functional as F
from torchvision import transforms
import torchvision
from PIL import Image
import dlib
import hopenet, utils

app = FastAPI()

# Load models globally
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = hopenet.Hopenet(torchvision.models.resnet.Bottleneck, [3, 4, 6, 3], 66)
saved_state_dict = torch.load("../model/hopenet_robust_alpha1.pkl", map_location=device)
model.load_state_dict(saved_state_dict)
model.eval()
model = model.to(device)
cnn_face_detector = dlib.cnn_face_detection_model_v1("../model/mmod_human_face_detector.dat")

transformations = transforms.Compose([
    transforms.Resize(224),
    transforms.CenterCrop(224),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
])

idx_tensor = torch.FloatTensor([idx for idx in range(66)]).to(device)

class OutputData(BaseModel):
    x_distance: float
    y_distance: float
    yaw: float
    area: float
    

@app.post("/face-detection", response_model=OutputData)
async def detect_face(file: UploadFile = File(...)):
    # Load image from upload
    try:
        image_bytes = await file.read()
        np_img = np.frombuffer(image_bytes, np.uint8)
        image = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
    except Exception as e:
        raise HTTPException(status_code=400, detail="Invalid image file")

    if image is None:
        raise HTTPException(status_code=400, detail="Failed to process image")

    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Detect faces
    dets = cnn_face_detector(image_rgb, 1)
    if len(dets) == 0:
        raise HTTPException(status_code=404, detail="No faces detected")

    # Select the face with the highest confidence
    max_conf = 0
    selected_det = None
    for det in dets:
        if det.confidence > max_conf:
            max_conf = det.confidence
            selected_det = det

    if not selected_det:
        raise HTTPException(status_code=404, detail="No high-confidence faces detected")

    x_min, y_min, x_max, y_max = selected_det.rect.left(), selected_det.rect.top(), selected_det.rect.right(), selected_det.rect.bottom()

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

    # Calculate distances
    center_x = (x_min + x_max) / 2
    center_y = (y_min + y_max) / 2
    x_distance = center_x / image.shape[1]
    y_distance = 1 - (center_y / image.shape[0])

    return OutputData(
        x_distance=x_distance,
        y_distance=y_distance,
        yaw=yaw_predicted.item(),
        area = (bbox_width * bbox_height)/(image.shape[0]*image.shape[1])
    )
