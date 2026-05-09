import torch
import soundfile as sf
from qwen_tts import Qwen3TTSModel

# Load the model
model = Qwen3TTSModel.from_pretrained(
    "/home/medical/Medical_Embodied/src/llm_node_py/Qwen3-TTS-12Hz-0.6B-CustomVoice",
    device_map="cuda:0",
    dtype=torch.bfloat16,
    # attn_implementation="flash_attention_2",
)

# Generate speech with specific instructions
wavs, sr = model.generate_custom_voice(
    text="其实我真的有发现，我是一个特别善于观察别人情绪的人。",
    language="Chinese", 
    speaker="Vivian",
    instruct="用特别愤怒的语气说", 
)

# Save the generated audio
sf.write("output_custom_voice.wav", wavs[0], sr)
