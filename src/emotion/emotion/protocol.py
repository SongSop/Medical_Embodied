"""Serial protocol for raspi-eye emotion control."""

HEADER = bytes((0xAA, 0x55))
CMD_SET_EMOTION = 0x01
MAX_PAYLOAD = 32

EMOTIONS = ['neutral', 'happy', 'sad', 'angry', 'surprised', 'curious']
EMOTION_BY_NAME = {name: i for i, name in enumerate(EMOTIONS)}


def xor_checksum(data: bytes) -> int:
    value = 0
    for b in data:
        value ^= b
    return value


def encode_set_emotion(emotion_id: int) -> bytes:
    if not 0 <= emotion_id < len(EMOTIONS):
        raise ValueError(f'emotion_id out of range: {emotion_id}')
    body = bytes((CMD_SET_EMOTION, 1, emotion_id))
    frame = HEADER + body
    return frame + bytes((xor_checksum(frame),))


def encode_emotion_name(name: str) -> bytes:
    key = name.strip().lower()
    if key not in EMOTION_BY_NAME:
        raise ValueError(f'unknown emotion: {name}')
    return encode_set_emotion(EMOTION_BY_NAME[key])
