import re

from unagifestival.tools.ps_controller.im920.model import IM920Response

HEX_BYTE_CHARACTER_COUNT = 2


def decode_frame(raw: str) -> IM920Response | None:
    """IM920 raw frameから完全なapplication textを復号する."""
    normalized = "".join(chr(ord(character) & 0x7F) for character in raw).strip()
    if ":" not in normalized:
        return None
    payload = re.sub(r"[^0-9A-Fa-f]", "", normalized.split(":", 1)[1])
    if (
        len(payload) < HEX_BYTE_CHARACTER_COUNT
        or len(payload) % HEX_BYTE_CHARACTER_COUNT
    ):
        return None
    try:
        text = bytes.fromhex(payload).decode("utf-8", errors="replace")
    except ValueError:
        return None
    return IM920Response(raw=normalized, text=text)
