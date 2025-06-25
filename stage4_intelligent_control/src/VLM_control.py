import os
import cv2 # cv2 is imported in VideoStreamThread
import time # time is imported in VideoStreamThread
import requests
import base64
# import ipdb
from zhipuai import ZhipuAI
# VideoStreamThread class from above should be defined here or imported
from video_streamer import VideoStreamThread
# --- 配置 ---
ZHIPUAI_API_KEY = "fdd16dc6b81c4381baca5af87f70c68b.hAFEHozKTmm8PNaK"
VIDEO_SOURCE = 1
STREAM_URL = "http://172.20.10.3:8080/?action=stream"
CONTROL_URL = "http://172.20.10.3:5000/control"

DEFAULT_USER_GOAL_PROMPT = "Do as you can see"
#DEFAULT_USER_GOAL_PROMPT = "Find a target that looks like a mineral water bottle and control the car to approach it as much as you can, it will be best if your camera is filled with the item. If the target is in the view, move forward"
SAVE_FRAMES_DIR = "saved_frames"
# FRAMES_TO_SKIP_FOR_CURRENTNESS is no longer needed with the threaded approach

# --- 初始化 ZhipuAI 客户端 ---
try:
    client = ZhipuAI(api_key=ZHIPUAI_API_KEY)
except Exception as e:
    print(f"Unable to initialize ZhipuAI client: {e}")
    exit()

# --- 初始化视频捕获线程 ---
video_thread = VideoStreamThread(STREAM_URL)
if not video_thread.cap.isOpened(): # Check if VideoCapture was successfully opened in thread init
    print("Failed to initialize VideoStreamThread due to VideoCapture error. Exiting.")
    exit()
video_thread.start() # Start the thread

print("Camera capture thread started and ZhipuAI client initialized successfully.")

if not os.path.exists(SAVE_FRAMES_DIR):
    os.makedirs(SAVE_FRAMES_DIR)
    print(f"Created directory: {SAVE_FRAMES_DIR}")

# --- 辅助函数 (encode_frame_to_base64, get_llm_command, send_command_to_car 保持不变) ---
def encode_frame_to_base64(frame):
    _, buffer = cv2.imencode('.jpg', frame)
    return base64.b64encode(buffer).decode('utf-8')

def get_visual_reasoning(client_zhipu, image_frame, goal_prompt_text, model_name="glm-4v-flash"):
    """
    第一步：调用多模态大模型描述图片，并根据目标做出初步推理。
    返回：包含分析结果的文本字符串。
    """
    base64_image = encode_frame_to_base64(image_frame) # encode_frame_to_base64 函数保持不变

    prompt_for_visual_model = f"""
Analyze this image carefully.
The overall mission is: '{goal_prompt_text}'

Based on the image and the mission, provide a concise textual description covering these aspects:
1.  Target Presence: Is the target (mineral water bottle) visible?
2.  Target Location: If visible, where is it in the frame (e.g., center, left, right, top-left)?
3.  Target Size/Distance: If visible, how large does it appear / how close is it (e.g., very small/far, medium size/distance, large/close, very large/filling a significant portion of view, filling entire view)?
4.  Clarity/Obstacles: Is the view clear? Are there any immediate obstacles?
5.  Overall Assessment: Briefly, what is the current situation relative to achieving the mission of getting extremely close to the target? For example, "Target is centered and close, ready for final approach," or "Target is to the left and far, requires turning and then moving forward," or "No target visible, need to search."

Keep your response as a single block of text. Do not suggest specific commands like FORWARD/LEFT/RIGHT/STOP yet.
"""
    messages = [
        {
            "role": "system",
            "content": "You are an intelligent visual analysis assistant helping a robot navigate by describing the scene relative to its goal."
        },
        {
            "role": "user",
            "content": [
                {
                    "type": "image_url",
                    "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}
                },
                {
                    "type": "text",
                    "text": prompt_for_visual_model
                }
            ]
        }
    ]

    try:
        api_start_time = time.time()
        response = client_zhipu.chat.completions.create(
            model=model_name, # 例如 "glm-4v-flash"
            messages=messages,
            max_tokens=300, # 给描述性文本留足够空间
            temperature=0.5  # 可以稍微高一点以获得更自然的描述
        )
        api_end_time = time.time()
        reasoning_text = response.choices[0].message.content.strip()
        print(f"[Step 1 - Visual Reasoning] LLM ({model_name}) call duration: {api_end_time - api_start_time:.2f} seconds")
        print(f"[Step 1 - Visual Reasoning] Output:\n{reasoning_text}\n")
        return reasoning_text
    except Exception as e:
        print(f"[Step 1 - Visual Reasoning] Error calling ZhipuAI API ({model_name}): {e}")
        return "Error: Could not get visual reasoning."

def get_action_from_reasoning(client_zhipu, 
                              reasoning_text, 
                              goal_prompt_text, 
                              command_history, # 新增参数：指令历史列表
                              model_name="glm-4"):
    """
    第二步：调用文本大模型，根据第一步的推理、目标、指令历史和行动规则，得到动作指令。
    返回：单个动作指令字符串。
    """
    
    # 格式化指令历史以便在Prompt中展示
    if not command_history:
        formatted_history = "No recent commands."
    else:
        # 例如: "[LEFT, FORWARD, LEFT]" (最新的在后面)
        formatted_history = f"[{', '.join(command_history)}]" 
        # 或者多行显示:
        # formatted_history = "\n".join([f"- {i+1}: {cmd}" for i, cmd in enumerate(command_history)])


    prompt_for_action_model = f"""
Situation Assessment from Visual Analysis:
---
{reasoning_text}
---

Overall Mission: '{goal_prompt_text}'

Recent Command History (oldest to newest, last {MAX_COMMAND_HISTORY_LENGTH} commands): 
---
{formatted_history}
---

Given the Situation Assessment, the Overall Mission, and the Recent Command History, select the single most appropriate command for the robot car. 
Critically consider the command history to avoid oscillations (e.g., repeatedly turning LEFT then RIGHT without progress) or getting stuck by repeating the same command if the situation doesn't improve as expected.

Command definitions:
- FORWARD: The target is centered in the view AND the car can still move closer to make the target appear significantly larger, aligning with the goal of having the target fill most of the view. Continue FORWARD if the target is centered but does not yet occupy a large majority of the camera view.
- LEFT: The target is primarily in the left portion of the camera's view.
- RIGHT: The target is primarily in the right portion of the camera's view.
- STOP: The target is extremely close (e.g., occupying most or all of the camera view, or a collision is imminent/touching), OR the objective of extreme closeness (target filling the view) has been achieved, OR if the command history suggests the current strategy is ineffective (e.g., stuck or oscillating).
- LOST: The target (e.g., mineral water bottle) is **not visible** in the current view, according to the Situation Assessment. This command indicates the need to initiate a search pattern or wait for the target to reappear.

Which single command word should be issued next? Your entire response must be strictly one of these words: FORWARD, LEFT, RIGHT, STOP, or LOST.
"""

    messages = [
        {
            "role": "system",
            "content": "You are a robot control logic unit. Your task is to choose a single action command based on the provided situation assessment, mission rules, and recent command history. Output only the command word. Avoid getting stuck or oscillating."
        },
        {
            "role": "user",
            "content": prompt_for_action_model
        }
    ]

    try:
        api_start_time = time.time()
        response = client_zhipu.chat.completions.create(
            model=model_name,
            messages=messages,
            max_tokens=10,
            temperature=0.05
        )
        api_end_time = time.time()
        command_output = response.choices[0].message.content.strip().upper()
        
        print(f"[Step 2 - Action Command] LLM ({model_name}) call duration: {api_end_time - api_start_time:.2f} seconds")
        
        valid_commands = ["FORWARD", "LEFT", "RIGHT", "STOP", "LOST"]
        final_command = "STOP" 
        
        if command_output in valid_commands:
            final_command = command_output
            print(f"[Step 2 - Action Command] LLM returned valid command: {final_command}")
        else:
            extracted = False
            for vc in valid_commands:
                if vc in command_output:
                    final_command = vc
                    extracted = True
                    print(f"[Step 2 - Action Command] LLM output '{command_output}', Extracted valid command: {final_command}")
                    break
            if not extracted:
                print(f"[Step 2 - Action Command] LLM output '{command_output}', returned invalid or unextractable command. Defaulting to {final_command}.")
        
        return final_command
    except Exception as e:
        print(f"[Step 2 - Action Command] Error calling ZhipuAI API ({model_name}): {e}")
        return "STOP"
    
def send_command_to_car(command, url):
    try:
        response = requests.post(url, json={'command': command}, timeout=2.0)
        response.raise_for_status()
        print(f"Successfully sent command: {command} to {url}")
        return True
    except requests.exceptions.Timeout:
        print(f"Failed to send command to {url} (Timeout after 2.0s)")
        return False
    except requests.exceptions.RequestException as e:
        print(f"Failed to send command to {url}: {e}")
        return False
    return False

# ... (之前的 import 和类/函数定义保持不变) ...
# VideoStreamThread 类应该已经定义或从其他文件导入
# import os, cv2, time, requests, base64, ZhipuAI, VideoStreamThread 等

# --- 主程序 ---
MAX_COMMAND_HISTORY_LENGTH = 5 # 您可以调整这个值

if __name__ == '__main__':
    current_active_goal = DEFAULT_USER_GOAL_PROMPT
    frame_count = 0
    command_history = [] # 初始化指令历史列表
    
    try:
        client = ZhipuAI(api_key=ZHIPUAI_API_KEY)
    except Exception as e:
        print(f"Unable to initialize ZhipuAI client: {e}")
        exit()

    print(f"\nCurrent active task goal: \"{current_active_goal}\"\n")
    print(f"Frames will be saved to '{SAVE_FRAMES_DIR}/' directory.")
    print(f"Command history will store last {MAX_COMMAND_HISTORY_LENGTH} commands.")

    video_thread = VideoStreamThread(STREAM_URL)
    if not video_thread.cap.isOpened():
        print("Failed to initialize VideoStreamThread. Exiting.")
        exit()
    video_thread.start()

    print("Camera capture thread started.")
    print("Waiting for the first frame from capture thread...")
    time.sleep(1.5) 

    try:
        while True:
            frame_ready, original_frame = video_thread.get_latest_frame()

            if not frame_ready or original_frame is None:
                time.sleep(0.05)
                if cv2.waitKey(1) & 0xFF == ord('q'): break
                continue 
            
            print("\n--- Processing new frame ---")
            print(f"Current command history: {command_history}") # 打印当前历史用于调试

            # === 第一步：获取视觉推理 ===
            reasoning_output = get_visual_reasoning(client, original_frame.copy(), current_active_goal)
            
            final_llm_command = "STOP" # 默认指令
            if "Error:" in reasoning_output:
                print("Skipping action due to error in visual reasoning.")
            else:
                # === 第二步：根据推理和指令历史获取行动指令 ===
                final_llm_command = get_action_from_reasoning(
                    client, 
                    reasoning_output, 
                    current_active_goal,
                    command_history # 传递当前指令历史
                )

            # --- 更新指令历史 ---
            command_history.append(final_llm_command)
            if len(command_history) > MAX_COMMAND_HISTORY_LENGTH:
                command_history.pop(0) # 保持历史列表的长度固定，移除最旧的指令
            # --------------------

            # ... (后续的代码：保存图片、发送指令给小车、显示等，与之前版本类似) ...
            # 在保存的图片上绘制最终指令
            frame_to_save = original_frame.copy()
            font_for_save = cv2.FONT_HERSHEY_SIMPLEX
            # ... (绘制文本的代码) ...
            cv2.putText(frame_to_save, f"LLM Cmd: {final_llm_command}", (10, frame_to_save.shape[0] - 25) , font_for_save, 0.6, (0,0,255), 1, lineType=cv2.LINE_AA)
            
            current_timestamp = time.strftime("%Y%m%d-%H%M%S")
            safe_command_for_filename = "".join(c if c.isalnum() else "_" for c in final_llm_command)
            frame_filename = os.path.join(SAVE_FRAMES_DIR, f"frame_{frame_count:04d}_{current_timestamp}_cmd_{safe_command_for_filename}.jpg")
            cv2.imwrite(frame_filename, frame_to_save)
            print(f"Saved frame with command: {frame_filename}")

            if final_llm_command == "LEFT" or final_llm_command == "RIGHT":
                send_command_to_car(final_llm_command, CONTROL_URL)
                time.sleep(0.1)
                send_command_to_car("STOP", CONTROL_URL)
            elif final_llm_command == "LOST":
                send_command_to_car("RIGHT", CONTROL_URL)
                time.sleep(0.2)
                send_command_to_car("STOP", CONTROL_URL)  
            elif final_llm_command == "FORWARD":
                send_command_to_car(final_llm_command, CONTROL_URL)
                time.sleep(0.25)
                send_command_to_car("STOP", CONTROL_URL)

            # 实时显示
            frame_for_display = original_frame 
            # ... (绘制显示文本的代码) ...
            cv2.putText(frame_for_display, f"Sent Cmd: {final_llm_command}", (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),2, lineType=cv2.LINE_AA)
            reasoning_display_line = reasoning_output.splitlines()[0] if reasoning_output and "Error:" not in reasoning_output else "No reasoning."
            cv2.putText(frame_for_display, f"AI Reason: {reasoning_display_line[:70]}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 255), 1, lineType=cv2.LINE_AA)
            cv2.putText(frame_for_display, f"History: {str(command_history)}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 255, 200), 1, lineType=cv2.LINE_AA)


            cv2.imshow('Live Feed with Command', frame_for_display)
            
            frame_count += 1

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Detected 'q' key press, exiting...")
                break
            
    finally:
        # ... (清理代码保持不变) ...
        print("Releasing resources...")
        if video_thread.is_alive():
            video_thread.stop()
            video_thread.join(timeout=5)
            if video_thread.is_alive(): print("[MainThread] Warning: Video capture thread did not terminate gracefully.")
        
        print("Attempting to send final STOP command.")
        send_command_to_car("STOP", CONTROL_URL) 
        cv2.destroyAllWindows()
        print("Program terminated.")