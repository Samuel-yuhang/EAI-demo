import time
import requests
from RealtimeSTT import AudioToTextRecorder

# --- 配置 ---
CONTROL_URL = "http://172.20.10.14:5000/control"  # 替换为你的小车控制地址
STT_LANGUAGE = "zh"  # 语音识别的语言 ("zh" 表示中文, "en" 表示英文)
# STT_MODEL = "small" # 可以选择STT模型大小，如 "tiny", "base", "small" 等，越小越快但可能牺牲精度

# 定义语音指令与小车动作的映射
# 您可以根据需要添加更多指令或修改关键词
COMMAND_MAP = {
    # 中文指令
    "前進": "FORWARD",
    "向前": "FORWARD",
    "后":"BACKWARD",
    "退":"BACKWARD",
    "走": "FORWARD",
    "左轉": "LEFT",
    "左转": "LEFT",
    "向左": "LEFT",
    "右轉": "RIGHT",
    "右转": "RIGHT",
    "向右": "RIGHT",
    "停止": "STOP",
    "停": "STOP",
    "停下": "STOP",
    # 英文指令
    "forward": "FORWARD",
    "go": "FORWARD",
    "move forward": "FORWARD",
    "left": "LEFT",
    "turn left": "LEFT",
    "right": "RIGHT",
    "turn right": "RIGHT",
    "stop": "STOP",
    "halt": "STOP",
}

# 小车实际接受的有效动作指令
VALID_CAR_ACTIONS = ["FORWARD", "LEFT", "RIGHT", "STOP", "BACKWARD"]

# --- 辅助函数 ---
def send_command_to_car(command, url):
    """将控制指令发送给小车"""
    if command not in VALID_CAR_ACTIONS:
        print(f"错误：无效的小车指令 '{command}'。允许的指令为: {VALID_CAR_ACTIONS}")
        return False
    try:
        response = requests.post(url, json={'command': command}, timeout=1.5) # 稍微增加超时
        response.raise_for_status() # 检查 HTTP 错误
        print(f"成功发送指令: {command} 到 {url}")
        return True
    except requests.exceptions.Timeout:
        print(f"发送指令到 {url} 超时 (指令: {command})")
        return False
    except requests.exceptions.RequestException as e:
        print(f"发送指令到 {url} 失败: {e} (指令: {command})")
        return False
    except Exception as e:
        print(f"发送指令 '{command}' 时发生未知错误: {e}")
        return False

def interpret_voice_command(text: str) -> str:
    """
    解释语音文本，并将其映射到预定义的小车动作。
    如果没有匹配到特定指令，默认为 "STOP" 以确保安全。
    """
    processed_text = text.lower().strip() # 转换为小写并去除首尾空格
    print(f"  正在解释: '{processed_text}'")

    # 为了更准确地匹配，可以优先匹配更长的指令短语
    # (如果您的 COMMAND_MAP 中有重叠的短语，例如 "go" 和 "go forward")
    # 但对于当前简单的关键词，直接遍历即可

    for voice_command_keyword, car_action in COMMAND_MAP.items():
        if voice_command_keyword.lower() in processed_text:
            print(f"  识别到关键词 '{voice_command_keyword}' -> 动作 '{car_action}'")
            return car_action # 返回第一个匹配到的指令

    print("  未识别到特定指令关键词，默认为 STOP。")
    return "STOP" # 如果没有关键词匹配，则返回 "STOP"

def stt_callback(text: str):
    """
    RealtimeSTT 的回调函数。当有新的语音转文本结果时，此函数会被调用。
    """
    print(f"\n[语音识别结果]: \"{text}\"")
    car_action = interpret_voice_command(text)
    
    # 即使是 "STOP" 指令，也发送，以确保小车根据最新语音输入行动
    send_command_to_car(car_action, CONTROL_URL)
    time.sleep(0.75)
    send_command_to_car("STOP", CONTROL_URL)

# --- 主程序 ---
if __name__ == '__main__':
    print("纯语音控制小车程序已启动。")
    print(f"小车控制地址: {CONTROL_URL}")
    print(f"语音识别语言: {STT_LANGUAGE}")
    print(f"可以说出指令如：“前进”、“左转”、“右转”、“停止”等。")
    print("按 Ctrl+C 退出程序。")

    stt_recorder = None
    try:
        # 初始化 RealtimeSTT AudioToTextRecorder
        # 您可以根据需要指定模型，例如 model=STT_MODEL
        stt_recorder = AudioToTextRecorder(language=STT_LANGUAGE)
        print("语音识别器已初始化完毕，正在聆听...")

        # 无限循环，持续处理语音输入
        # recorder.text(callback) 会阻塞，直到有语音片段被处理完毕，
        # 然后调用回调函数，之后此函数返回，外层 while True 再次调用它。
        while True:
            stt_recorder.text(stt_callback)
            # 可以在这里加入一个微小的延时，如果STT返回过快或者CPU占用高
            # time.sleep(0.05) 

    except KeyboardInterrupt:
        print("\n检测到 Ctrl+C，正在关闭程序...")
    except Exception as e:
        print(f"程序运行过程中发生未预料的错误: {e}")
    finally:
        if stt_recorder:
            print("正在停止语音识别器...")
            stt_recorder.stop()
            print("语音识别器已停止。")
        
        # 在程序退出前，发送最后的停止指令以确保安全
        print("发送最终停止指令到小车...")
        send_command_to_car("STOP", CONTROL_URL)
        print("程序已安全退出。")