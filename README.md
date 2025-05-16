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

## 9 · Use case demos (videos)

We have recorded **three demo videos** demonstrating the main use cases of RUMI in realistic conditions.  
The interface was run on a standard computer for recording purposes, but the system is fully compatible with any robot like **Sancho**, and would behave identically when deployed there.

---

### Case A – Pre‑registration of users  
The operator registers an annotated face image of a guest **before** they interact with the robot. When the guest appears, the system recognizes them immediately without needing any further action.  
[![Case A](https://img.youtube.com/vi/0R9K5xnG_uo/0.jpg)](https://youtu.be/0R9K5xnG_uo)

---

### Case B – Statistical logging without touching the HRI system  
During a full day of interactions, the system automatically groups detections into sessions and logs them for analysis. Statistics and session data can be accessed **at any time via the API or the web interface**, without manual intervention.  
[![Case B](https://img.youtube.com/vi/ZGIwOdBQW1Y/0.jpg)](https://youtu.be/ZGIwOdBQW1Y)

---

### Case D – Safe online editing of the identity database  
The operator renames and deletes faceprints **while the robot is still running**, with no need to restart nodes or access the internal HRI database.  
[![Case D](https://img.youtube.com/vi/V8bheVLriKw/0.jpg)](https://youtu.be/V8bheVLriKw)

---

### What about Case C – Remote supervision with live video?

**Case C – "The operator can see what the robot sees remotely" –** is **inherently demonstrated in all three videos above**:  
In each demo, the web interface displays the real-time video stream exposed by the camera, along with face detection overlays.  
This confirms that RUMI can be used from any device with network access, without needing physical proximity to the robot.

---

The videos are hosted on YouTube as *unlisted* links and can be accessed from this section.  
The link to the paper will be added here once it becomes available.
