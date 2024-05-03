#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import fastapi
import uvicorn
import pydantic
from starlette.middleware.cors import CORSMiddleware
import asyncio
import json

app = fastapi.FastAPI(debug=True)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class Mode(pydantic.BaseModel):
    name: str
    comment: str

last_mode = Mode(name="unknown", comment="none")

@app.get("/mode")
async def get_mode():
    return last_mode

@app.post("/mode")
async def post_mode(mode: Mode):
    global last_mode
    print("mode_request", mode.name, mode.comment)
    last_mode = mode
    return last_mode

@app.websocket("/mode")
async def websocket_endpoint(websocket: fastapi.WebSocket):
    try:
        await websocket.accept()
        print("connetct", websocket.headers.get("sec-websocket-key"))
        while True:
            mode_dic = {"name":last_mode.name, "comment":last_mode.comment}
            await websocket.send_text(json.dumps(mode_dic))
            await asyncio.sleep(0.5)
    except Exception as e:
        print("disconnected:", e)

def main(argv=sys.argv):
    uvicorn.run(app, host="", port=8010, log_level="warning")

if __name__ == "__main__":
    main(sys.argv)
