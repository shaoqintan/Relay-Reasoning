import json

SCENE_DB = "/Users/shubhamrathod/PycharmProjects/googleResoning/memory/scene_descriptions.jsonl"

def load_latest_scene() -> str:
    """
    Reads the last non-empty line of a JSONL file and returns its 'description' field.
    Returns empty string if the file doesn’t exist, is empty, or can’t be parsed.
    """
    try:
        last_line = None
        with open(SCENE_DB, "r") as f:
            for line in f:
                if line.strip():
                    last_line = line
        if not last_line:
            return ""
        entry = json.loads(last_line)
        return entry.get("description", "")
    except FileNotFoundError:
        # No file yet
        return ""
    except json.JSONDecodeError as e:
        print(f"JSON parse error: {e}")
        return ""

# usage
scene_desc = load_latest_scene()
print("Latest scene description:", scene_desc)
