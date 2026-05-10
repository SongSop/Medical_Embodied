import torch
import pyaudio
import numpy as np
from qwen_tts import Qwen3TTSModel

# 1. 加载模型
model = Qwen3TTSModel.from_pretrained(
    "/home/medical/Medical_Embodied/src/llm_node_py/Qwen3-TTS-12Hz-0.6B-CustomVoice",
    device_map="cuda:0",
    dtype=torch.bfloat16,
)

# 2. 生成语音
wavs, sr = model.generate_custom_voice(
    text="其实我真的有发现，我是一个特别善于观察别人情绪的人。",
    language="Chinese",
    speaker="Vivian",
    instruct="用特别愤怒的语气说",
)

# 3. 初始化 PyAudio 播放器
p = pyaudio.PyAudio()

# 4. 打开播放流
stream = p.open(format=pyaudio.paFloat32,  # 因为 wavs[0] 是 float32
                channels=1,               # 单声道
                rate=sr,                  # 采样率
                output=True)

# 5. 播放音频
stream.write(wavs[0].astype(np.float32).tobytes())

# 6. 停止并关闭流
stream.stop_stream()
stream.close()
p.terminate()