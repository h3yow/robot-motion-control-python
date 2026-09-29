"""本地机器人多关节运动控制服务入口。

运行：uv run python main.py  （或 uvicorn main:app）
"""

import uvicorn

from robot_motion.api import app  # noqa: F401

if __name__ == "__main__":
    uvicorn.run("main:app", host="127.0.0.1", port=8000)
