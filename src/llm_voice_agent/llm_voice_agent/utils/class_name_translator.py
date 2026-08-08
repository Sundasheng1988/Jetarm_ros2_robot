CLASS_NAME_MAP = {
    "cup": "杯子",
    "bottle": "瓶子",
    "phone": "手机",
    "cell phone": "手机",
    "cube": "方块",
    "block": "方块",
    "ball": "球",
    "person": "人",
    "cat": "猫",
    "dog": "狗",
    "keyboard": "键盘",
    "monitor": "显示器",
}

def translate_class_name(name: str) -> str:
    if not name:
        return name
    key = name.strip().lower()
    return CLASS_NAME_MAP.get(key, name)
