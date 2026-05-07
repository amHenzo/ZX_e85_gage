from pathlib import Path

Import("env")

PROJECT_DIR = Path(env.subst("$PROJECT_DIR"))
DATA_DIR = PROJECT_DIR / "data"
OUT_FILE = PROJECT_DIR / "src" / "generated_web_assets.h"


def raw_string(name, path):
    text = path.read_text(encoding="utf-8")
    delimiter = "ZXWEB"
    while f"){delimiter}\"" in text:
        delimiter += "X"
    return f'const char {name}[] PROGMEM = R"{delimiter}({text}){delimiter}";\n'


content = """#ifndef GENERATED_WEB_ASSETS_H
#define GENERATED_WEB_ASSETS_H

#include <Arduino.h>

"""
content += raw_string("WEB_INDEX_HTML", DATA_DIR / "index.html")
content += raw_string("WEB_STYLE_CSS", DATA_DIR / "style.css")
content += raw_string("WEB_APP_JS", DATA_DIR / "app.js")
content += "\n#endif\n"

OUT_FILE.write_text(content, encoding="utf-8")
