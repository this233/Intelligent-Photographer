import os
import subprocess
import time
import requests
import json
import datetime
from PIL import Image
from flask import Flask, render_template, request, jsonify
import threading

DEBUG = False
IS_ROBOT = True
if not DEBUG:
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    import rospy

API_KEY = 'app-SEI9dset09y2kQb1nhCKwBZ9'

app = Flask(__name__)

iterations_data = []
condition = threading.Condition()
continue_iteration = threading.Event()
stop_iteration = threading.Event()

def start_camera():
    print("Starting camera...")
    subprocess.run(['adb', 'shell', 'am', 'start', '-a', 'android.media.action.STILL_IMAGE_CAMERA'])
    time.sleep(2)

def take_picture():
    print("Taking picture...")
    subprocess.run(['adb', 'shell', 'input', 'keyevent', '27'])
    time.sleep(3)

today = datetime.date.today().strftime("%Y%m%d")

def get_latest_photo():
    print('Getting latest photo...')
    result = subprocess.run(['adb', 'shell', 'ls', '-lt', f'/sdcard/DCIM/Camera/IMG_{today}_*.jpg'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    photos = result.stdout.decode().splitlines()
    if photos:
        latest_photo = photos[0].split()[-1]
        return latest_photo
    return None

def download_photo(photo_path, save_path):
    print(f"Downloading photo from {photo_path}...")
    subprocess.run(['adb', 'pull', photo_path, save_path])

def delete_photo(photo_path):
    print(f"Deleting photo {photo_path} from device...")
    subprocess.run(['adb', 'shell', 'rm', photo_path])

def compress_image(image_path, output_path=None, scale_factor=5):
    if output_path is None:
        output_path = os.path.join('static', 'latest_image.jpg')
    with Image.open(image_path) as img:
        width, height = img.size
        new_width, new_height = width // scale_factor, height // scale_factor
        if not IS_ROBOT:
            img = img.resize((new_width, new_height), Image.Resampling.LANCZOS)
        else:
            img = img.resize((new_width, new_height), Image.ANTIALIAS)
        img.save(output_path)
        print(f"Image compressed and saved to {output_path}")

def upload_file(file_path, user):
    upload_url = "https://api.dify.ai/v1/files/upload"
    headers = {"Authorization": f"Bearer {API_KEY}"}
    try:
        with open(file_path, 'rb') as file:
            suffix = file_path.split('.')[-1].lower()
            suffix = 'jpeg' if suffix == 'jpg' else suffix
            if suffix not in ["png", "jpeg"]:
                print("Unsupported file type.")
                return None
            files = {'file': (os.path.basename(file_path), file, f'image/{suffix}')}
            data = {"user": user, "type": "TXT"}
            proxies = {
                "http": None,
                "https": None
            }
            response = requests.post(upload_url, headers=headers, files=files, data=data,proxies=proxies)
            if response.status_code == 201:
                print("File uploaded successfully.")
                return response.json().get("id")
            print(f"File upload failed. Status code: {response.status_code}")
    except Exception as e:
        print(f"Error: {str(e)}")
    return None

def run_workflow(file_id, user, now_Yaw, now_x, now_y,now_area, response_mode="blocking"):
    workflow_url = "https://api.dify.ai/v1/workflows/run"
    headers = {
        "Authorization": f"Bearer {API_KEY}",
        "Content-Type": "application/json"
    }
    data = {
        "inputs": {
            "now_image": {
                "transfer_method": "local_file",
                "upload_file_id": file_id,
                "type": "image"
            },
            "now_Yaw": now_Yaw,
            "now_x": now_x,
            "now_y": now_y,
            "now_area": now_area,
        },
        "response_mode": response_mode,
        "user": user
    }
    try:
        print("运行工作流...")
        proxies = {
            "http": None,
            "https": None
        }
        response = requests.post(workflow_url, headers=headers, json=data,proxies=proxies)
        if response.status_code == 200:
            print("工作流执行成功")
            print(response.json())
            return response.json()
        else:
            print(f"工作流执行失败，状态码: {response.status_code}")
            exit(0)
    except Exception as e:
        print(f"发生错误: {str(e)}")
        exit(0)



def get_xy_yaw_area(file_path):
    url = "https://1e20-58-60-1-29.ngrok-free.app/face-detection"
    with open(file_path, 'rb') as image_file:
        response = requests.post(url, files={"file": image_file}, proxies={"http": None, "https": None})
    if response.status_code == 200:
        return response.json().get("x_distance"), response.json().get("y_distance"), response.json().get("yaw"), response.json().get("area")
    else:
        print(f"Error: {response.status_code}")
        return None, None, None, None

def robot_processing(now_height, now_Yaw):
    global iterations_data
    for _ in range(10):
        start_camera()
        take_picture()
        latest_photo = get_latest_photo()
        if not latest_photo:
            print("No photo found.")
            break
        save_path = os.path.join('static', os.path.basename(latest_photo))
        download_photo(latest_photo, save_path)
        delete_photo(latest_photo)
        compressed_path = os.path.join('static', 'latest_image.jpg')
        compress_image(save_path, compressed_path)
        x, y, yaw, area = get_xy_yaw_area(compressed_path)
        print('get_xy_yaw_area: ',x, y, yaw, area)
        file_id = upload_file(compressed_path, "difyuser")
        if not file_id:
            print("Failed to upload file.")
            break
        result = run_workflow(file_id, "difyuser", yaw, x, y, area)
        print(result["data"]["outputs"])
        next_Yaw, next_x, next_y, next_area_act,text = result["data"]["outputs"].values()
        init_face_x_delta = (next_Yaw - yaw) / 10 * 0.25 # TODO area
        inter = {
            "face_Yaw": yaw,
            "next_face_Yaw": next_Yaw,
            "now_area": area,
            "init_face_x_delta":init_face_x_delta, # TODO area
            "face_y_delta": next_y - y,
            "face_x_delta": next_x - (x+init_face_x_delta),
            "camera_height_delta": (-15) * (next_y - y)
        }
        
        output = {
            "actural_camera_yaw_delta": -1 * (next_Yaw - yaw)*0.8 ,#-5 if -1 * (next_Yaw - yaw) < 0 else -1 * (next_Yaw - yaw)+5,
            "actural_camera_height_delta": max(0, min(6, now_height + inter["camera_height_delta"])) - now_height,
            "actural_camera_x_delta": (-10) * inter["face_x_delta"],
            # "actural_camera_y_delta": 2 if next_area_act == "更大" else (-2 if next_area_act == "更小" else 0)
            "actural_camera_y_delta": -(area - 0.016)/0.004
        }
        iteration = {
            "image_url": f'./static/iteration_{len(iterations_data)}.jpg',
            "text": text,
            "now_height": now_height,
            "now_Yaw": now_Yaw,
            "face_Yaw": yaw,
            "face_x": x,
            "face_y": y,
            "face_area": area,
            "next_face_Yaw": next_Yaw,
            "next_face_x": next_x,
            "next_face_y": next_y,
            "next_face_area_act": next_area_act,
            "inter": inter,
            "output": output
        }
        iterations_data.append(iteration)
        compress_image(save_path, iteration["image_url"])
        
        now_Yaw += int(output["actural_camera_yaw_delta"]/10)
        now_height += int(output["actural_camera_height_delta"])
        if not DEBUG:
            # 发送控制指令给机器人
            # 向零取整
            vel_info.linear.z = int(output["actural_camera_height_delta"]) # 向零取整
            vel_info.angular.z = output["actural_camera_yaw_delta"]/10 # 向零取整
            vel_info.linear.y = output["actural_camera_x_delta"]
            vel_info.linear.x = output["actural_camera_y_delta"]
            
            vel_pub.publish(vel_info)
        
        continue_iteration.clear()
        stop_iteration.clear()
        while not (continue_iteration.is_set() or stop_iteration.is_set()):
            time.sleep(0.1)
        if stop_iteration.is_set():
            break

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/iterations')
def get_iterations():
    # print(f"Current iterations_data: {iterations_data}")  # 输出调试信息
    return jsonify(iterations_data)

@app.route('/control', methods=['POST'])
def control():
    action = request.form.get('action')
    with condition:
        if action == 'continue':
            continue_iteration.set()
        elif action == 'stop':
            stop_iteration.set()
    return '', 204

if __name__ == "__main__":
    if not DEBUG:
        rospy.init_node('info')
        vel_info=Twist()
        vel_pub = rospy.Publisher('/info', Twist, queue_size=10)
    now_height = int(input("Enter initial height: "))
    now_Yaw = int(input("Enter initial yaw: "))
    processing_thread = threading.Thread(target=robot_processing, args=(now_height, now_Yaw))
    processing_thread.start()
    app.run(host='0.0.0.0', port=5000)
