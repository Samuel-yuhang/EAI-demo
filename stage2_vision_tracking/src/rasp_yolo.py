import os
import cv2
import time
import argparse
import requests
import torch
import model.detector
import utils.utils

stream_url = "http://172.20.10.14:8080/?action=stream"    # 这里请替换为你的树莓派 IP 地址
control_url = "http://172.20.10.14:5000/control"          # 这里请替换为你的树莓派 IP 地址
cap = cv2.VideoCapture(stream_url)                       # 获取视频的输入源，也就是这个网址

cfg = utils.utils.load_datafile('data/coco.data')
weights = 'modelzoo/coco2017-0.241078ap-model.pth'
assert os.path.exists(weights), "请指定正确的模型路径"

target_categories = ["person"]                           # 我们这里要追踪人，你可以换成其他物体

# 模型加载：如果你的电脑上有 GPU，可以用第一行；如果你的电脑有 Apple M 系列芯片，可以用第二行
# device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
# device = torch.device("mps" if torch.backends.mps.is_available() else "cpu")
device = "cpu"

model = model.detector.Detector(cfg["classes"], cfg["anchor_num"], True).to(device)
model.load_state_dict(torch.load(weights, map_location=device))
model.eval()

def send_command(box, h, w):
    x_center = (box[0] + box[2]) / 2 / w                          # 检测框的中心点
    box_area = (box[2] - box[0]) * (box[3] - box[1]) / (h * w)    # 检测框的面积
    print("box area: ", box_area)
    print("x_center: ", x_center)
    # 如果已经跑到人跟前了，就停下来（否则就创上去了，狂暴创人小车石锤）
    if box_area > 0.7:      
        response = requests.post(control_url, json={'command': "STOP"})
        print("Stop")
    else:
        # 如果人的检测框的中心点在左边，就左转
        if x_center < 0.3:
            response = requests.post(control_url, json={'command': "LEFT"})            
            print("Turn left")
        # 如果人的检测框的中心点在右边，就右转
        elif x_center > 0.7:
            response = requests.post(control_url, json={'command': "RIGHT"})            
            print("Turn right")
        # 如果人的检测框的中心点在中间，就前进
        else:
            response = requests.post(control_url, json={'command': "FORWARD"})            
            print("Go straight")
    

while True:
    ret, frame = cap.read()                             # 不停地从输入源读取一帧图像
    if not ret:                                         # 如果读取失败，结束循环
        break
    # 将图片处理成 YOLO 模型的输入
    res_img = cv2.resize(frame, (cfg["width"], cfg["height"]), interpolation = cv2.INTER_LINEAR) 
    img = res_img.reshape(1, cfg["height"], cfg["width"], 3)
    img = torch.from_numpy(img.transpose(0, 3, 1, 2))
    img = img.to(device).float() / 255.0
    # 运行 YOLO 模型，得到输出
    preds = model(img)
    # 对输出进行后处理，例如过滤掉重叠的框和置信度低的框
    output = utils.utils.handel_preds(preds, cfg, device)
    output_boxes = utils.utils.non_max_suppression(output, conf_thres = 0.3, iou_thres = 0.4)

    #加载label names
    LABEL_NAMES = []
    with open(cfg["names"], 'r') as f:
        for line in f.readlines():
            LABEL_NAMES.append(line.strip())
    h, w, _ = frame.shape
    scale_h, scale_w = h / cfg["height"], w / cfg["width"]

    if len(output_boxes[0]) == 0:
        response = requests.post(control_url, json={'command': "STOP"})
        print("Stop")

    #绘制预测框
    for box in output_boxes[0]:
        box = box.tolist()
       
        obj_score = box[4]
        category = LABEL_NAMES[int(box[5])]

        x1, y1 = int(box[0] * scale_w), int(box[1] * scale_h)
        x2, y2 = int(box[2] * scale_w), int(box[3] * scale_h)
        
        if category in target_categories:
            send_command(box, cfg["height"], cfg["width"])
        else:
            response = requests.post(control_url, json={'command': "STOP"})
            print("Stop")

        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 2)
        cv2.putText(frame, '%.2f' % obj_score, (x1, y1 - 5), 0, 0.7, (0, 255, 0), 2)    
        cv2.putText(frame, category, (x1, y1 - 25), 0, 0.7, (0, 255, 0), 2)
    processed_frame = frame
    # 显示图像
    cv2.imshow('Processed Frame', processed_frame)      # 显示获取到的图像

    end = time.perf_counter()
    
    if cv2.waitKey(1) & 0xFF == ord('q'):               # 检测 q 键有没有按下，按下就退出程序
        break

cap.release()                                           # 释放视频输入源
cv2.destroyAllWindows() 