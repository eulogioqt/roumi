<div align="center">

# RUMI  
**R**OS **U**ser **M**anagement **I**nterface  
Light‑weight web tool for monitoring and editing face‑recognition data in HRI systems

</div>

---

## 1 · Overview  
RUMI lets an operator **see, edit and analyse** the identities managed by a face‑recognition
pipeline **without stopping the robot**.  
Everything runs on **ROS 2**, **FastAPI** and **React**; only one topic and one service are mandatory.

* **Session manager** – groups detections into sessions and stores them in SQLite  
* **REST API** – CRUD for identities + session queries (port 7654)  
* **ros2web bridge** – native ROS 2 ↔ WebSocket (port 8765)  
* **Web UI** – live monitoring and editing (port 8173)

---

## 2 · Quick start  
```
git clone https://github.com/eulogioqt/rumi.git
cd rumi/ros2_ws
colcon build
source install/setup.bash
ros2 launch rumi_web launch.py
```

If the browser does not open automatically:

| Component          | URL                                           |
|--------------------|-----------------------------------------------|
| Web interface      | http://localhost:8173                         |
| REST docs (Swagger)| http://localhost:7654/api/hri/docs            |
| ReDoc              | http://localhost:7654/api/hri/redoc           |
| Raw OpenAPI        | http://localhost:7654/api/hri/openapi.json    |
| WebSocket          | ws://localhost:8765                           |

---

## 3 · Connecting your detector  

### 3.1 Topic to publish  
`/rumi/sessions/process` - `rumi_msgs/msg/SessionMessage`  

```
uint32  faceprint_id
float64 detection_score
float64 classification_score
```

### 3.2 Runtime parameters  

| Service | Type | Fields |
|---------|------|--------|
| /rumi/sessions/set_params | rumi_msgs/srv/SetSessionParams | timeout_seconds, time_between_detections |

---

## 4 · Faceprint Interface – plug in your DB  

Edit **`rumi_web/faceprint_api.py`** and implement five methods:

```
get_all_faceprints()             → JSONResponse / HTTPException
get_faceprint(id)                → …
create_faceprint(name,img64)     → …
update_faceprint(id,payload)     → …
delete_faceprint(id)             → …
```

Return `JSONResponse` on success or `HTTPException` on error.  
Your code may call ROS 2 services, internal REST endpoints or any database — RUMI only verifies the interface.

---

## 5 · Realtime bridge (ros2web)

* Default topics via launch param  

      topics="[ ['/camera/bbox_image','IMAGE'] ]"

* Or at run‑time  

      ros2 service call /ros2web/subscribe ros2web_msgs/srv/R2WSubscribe \
          "{topic: '/my/topic', name: 'ALIAS'}"

### Faceprint events (keep UI in sync)

```json
{
  "type": "FACEPRINT_EVENT",
  "data": { 
    "event": "CREATE", 
    "id": 42 
  }
}
```
Note: Event type can be `CREATE`, `UPDATE` or `DELETE`.

Publish JSON to /ros2web/ros; the Web UI refreshes automatically.

---

## 6 · Session manager internals  

Key hooks:

| Interface | Purpose |
|-----------|---------|
| /rumi/sessions/process (msg) | feed detections |
| /rumi/sessions/get     (srv) | query sessions |
| /rumi/sessions/set_params   (srv) | adjust timeout_seconds / time_between_detections |

Sessions are stored in `rumi_web/database/system.db` (SQLite).  
Grouping and statistics are fully automatic.

---

## 7 · Customising the Web UI  

A production build is shipped in `client/dist/` — Node JS is **not** required.  
If you want to tweak it:

```
cd client
npm install
npm run build
```
---

## 8 · Port summary

| Purpose      | Port |
|--------------|------|
| REST API     | 7654 |
| WebSocket    | 8765 |
| Web frontend | 8173 |

---