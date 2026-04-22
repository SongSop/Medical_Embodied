from pathlib import Path


def get_dashscope_key() -> str:
    current_dir = Path(__file__).resolve().parent
    key_path = current_dir / "../key/dashscope.key"
    return key_path.resolve().read_text(encoding="utf-8").strip()
