"""Loopback-only browser rendering of native MuJoCo 4 simulation state.

This is a small local viewer, not MuJoCo Studio's WebViewer (the 4.0 Windows
wheel lacks its headless_ui extension). Physics and official IK stay in Python.
Only compiled mesh geometry and live geom transforms are sent to the browser.
"""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
import time
import webbrowser

PAGE = r'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RM65 · 官方 IK 仿真</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#10151e;color:#eef3fb;font:14px system-ui,"Microsoft YaHei",sans-serif;overflow:hidden}
canvas{display:block}header{position:fixed;top:24px;left:28px;right:28px;display:flex;align-items:center;justify-content:space-between;pointer-events:none}
h1{font-size:23px;font-weight:600;letter-spacing:.02em;margin:0 0 7px}small{color:#9badc6;font-size:13px}.tag{color:#83dcc4;border:1px solid #335d57;padding:8px 12px;border-radius:20px;background:#142a2a}
.panel{position:fixed;left:28px;bottom:28px;width:290px;padding:20px;border:1px solid #354052;border-radius:15px;background:#17202ee8;backdrop-filter:blur(12px)}
.line{display:flex;justify-content:space-between;margin:12px 0;color:#a8b7cb}.value{font-variant-numeric:tabular-nums;color:#eaf2ff}
#state{font-size:18px;color:#83dcc4;margin-bottom:18px}.dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:9px;background:currentColor}
.orange{color:#ff8b54}.cyan{color:#49d9ed}.legend{margin-top:15px;line-height:1.8;color:#bdc9db}footer{position:fixed;bottom:30px;right:28px;color:#9badc6;text-align:right;line-height:1.8}
.buttons{position:fixed;right:28px;top:112px;display:flex;gap:8px}button{border:1px solid #42516a;background:#202c40;color:#eaf2ff;padding:10px 15px;border-radius:9px;cursor:pointer}button:hover{background:#2d405c}#error{color:#ff9f87;max-width:250px;font-size:12px;margin-top:10px}
</style>
<header><div><h1>RM65 连续跟随</h1><small>官方 IK · MuJoCo 4.0 · 本机物理仿真</small></div><div class="tag">纯仿真 · 未连接机械臂</div></header>
<div class="buttons"><button onclick="action('pause')">暂停</button><button onclick="action('resume')">继续</button><button onclick="action('reset')">重新开始</button></div>
<div class="panel"><div id="state">正在加载模型…</div><div class="line"><span>仿真时间</span><span class="value" id="time">—</span></div><div class="line"><span>当前位置 / m</span></div><div class="value" id="xyz">—</div><div class="line"><span>当前位置误差</span><span class="value" id="err">—</span></div><div class="legend"><span class="orange">●</span> 目标 TCP　<span class="cyan">●</span> 实际 TCP<br>自动演示：平移 → 转动 → 松开保持</div><div id="error"></div></div>
<footer>左键旋转视角 · 滚轮缩放 · 右键平移<br>执行器参数为仿真假设，不代表真机性能</footer>
<script type="importmap">{"imports":{"three":"/static/three.module.js"}}</script>
<script>window.action=async function(action){await fetch('/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action})})};</script>
<script type="module">
import * as THREE from 'three';import {OrbitControls} from '/static/OrbitControls.js';
const renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.setSize(innerWidth,innerHeight);renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;document.body.prepend(renderer.domElement);
const scene=new THREE.Scene();scene.background=new THREE.Color('#101925');const camera=new THREE.PerspectiveCamera(42,innerWidth/innerHeight,.01,30);camera.up.set(0,0,1);camera.position.set(1.3,-1.6,1.05);
const controls=new OrbitControls(camera,renderer.domElement);controls.target.set(0,0,.45);controls.enableDamping=true;controls.minDistance=.35;controls.maxDistance=4;controls.update();
scene.add(new THREE.HemisphereLight(0xc4d9ff,0x303c4e,2));const light=new THREE.DirectionalLight(0xffffff,3);light.position.set(1,-2,3);light.castShadow=true;light.shadow.mapSize.set(2048,2048);light.shadow.camera.left=-1.5;light.shadow.camera.right=1.5;light.shadow.camera.top=1.5;light.shadow.camera.bottom=-1.5;light.shadow.normalBias=.001;scene.add(light);
const grid=new THREE.GridHelper(4,40,0x3b526e,0x23344b);grid.rotation.x=Math.PI/2;grid.position.z=-.023;scene.add(grid);
const objects=[];let connected=false;
try{
 const model=await(await fetch('/model')).json();const meshes=model.meshes.map(v=>{const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(v.vertices.flat(),3));g.setIndex(v.faces.flat());g.computeVertexNormals();return g});
 for(const geom of model.geoms){let geometry;if(geom.type===7)geometry=meshes[geom.mesh];else if(geom.type===0)geometry=new THREE.PlaneGeometry(4,4);else if(geom.type===2)geometry=new THREE.SphereGeometry(geom.size[0],24,16);else continue;
  const c=geom.rgba;const material=new THREE.MeshStandardMaterial({color:new THREE.Color(c[0],c[1],c[2]),roughness:.48,metalness:.12,transparent:c[3]<1,opacity:c[3],side:THREE.DoubleSide});if(geom.type===0)material.color.set('#1d2a3b');
  const mesh=new THREE.Mesh(geometry,material);mesh.matrixAutoUpdate=false;mesh.castShadow=geom.type===7;mesh.receiveShadow=true;scene.add(mesh);objects.push({id:geom.id,mesh});}
 const actual=new THREE.Mesh(new THREE.SphereGeometry(.008,20,14),new THREE.MeshBasicMaterial({color:0x49d9ed}));scene.add(actual);
 async function update(){try{const state=await(await fetch('/state',{cache:'no-store'})).json();for(const {id,mesh} of objects){const p=state.geoms[id].p,r=state.geoms[id].r;mesh.matrix.set(r[0],r[1],r[2],p[0],r[3],r[4],r[5],p[1],r[6],r[7],r[8],p[2],0,0,0,1)}actual.position.fromArray(state.tcp_world);
  document.getElementById('time').textContent=state.time.toFixed(2)+' s';document.getElementById('xyz').textContent=state.tcp.map(x=>x.toFixed(4)).join('　');document.getElementById('err').textContent=(state.error_m*1000).toFixed(2)+' mm';
  document.getElementById('state').textContent=state.paused?'已暂停':({tracking:'正在连续跟随',anchoring:'建立跟随起点',idle:'松开保持',hold:'短暂保持',fault:'计算已停止'}[state.state]||'正在启动');document.getElementById('error').textContent=state.state==='fault'?state.reason:'';connected=true;
 }catch(e){document.getElementById('state').textContent='仿真服务连接中断';document.getElementById('error').textContent=String(e)}setTimeout(update,40)}update();
}catch(e){document.getElementById('error').textContent=String(e)}
addEventListener('resize',()=>{camera.aspect=innerWidth/innerHeight;camera.updateProjectionMatrix();renderer.setSize(innerWidth,innerHeight)});
function draw(){requestAnimationFrame(draw);controls.update();renderer.render(scene,camera)}draw();
</script></html>'''


def model_payload(sim):
    model = sim.model
    meshes = []
    for index in range(model.nmesh):
        start, count = model.mesh_vertadr[index], model.mesh_vertnum[index]
        face_start, face_count = model.mesh_faceadr[index], model.mesh_facenum[index]
        meshes.append({"vertices": model.mesh_vert[start:start + count].tolist(),
                       "faces": model.mesh_face[face_start:face_start + face_count].tolist()})
    # URDF import places visuals in group 1. Collision copies are not drawn twice.
    geoms = [{"id": i, "type": int(model.geom_type[i]), "mesh": int(model.geom_dataid[i]),
              "size": model.geom_size[i].tolist(), "rgba": model.geom_rgba[i].tolist()}
             for i in range(model.ngeom) if model.geom_group[i] == 1 or model.geom_type[i] in (0, 2)]
    return {"meshes": meshes, "geoms": geoms, "engine": sim.mujoco.__version__}


def serve(sim, status_path, port, assets, *, open_browser=True):
    allowed = {name: assets / name for name in ("three.module.js", "three.core.js", "OrbitControls.js")}
    if not all(path.is_file() for path in allowed.values()):
        raise FileNotFoundError("The pinned local three.js viewer assets are missing")
    lock, stopping = threading.RLock(), threading.Event()
    state = {"paused": False}
    static_model = json.dumps(model_payload(sim), separators=(",", ":")).encode()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, data, content_type="application/json", code=200):
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            if self.path == "/":
                return self.reply(PAGE.encode(), "text/html; charset=utf-8")
            if self.path == "/model":
                return self.reply(static_model)
            if self.path.startswith("/static/"):
                path = allowed.get(self.path.removeprefix("/static/"))
                return self.reply(path.read_bytes(), "text/javascript") if path else self.reply(b"{}", code=404)
            if self.path == "/state":
                with lock:
                    pose = sim.measured_pose()
                    payload = {"time": float(sim.data.time), "paused": state["paused"],
                               "state": sim.result.state if sim.result else "idle",
                               "reason": sim.result.reason if sim.result else "",
                               "tcp": pose.xyz.tolist(), "tcp_world": sim.data.xpos[sim.tcp].tolist(),
                               "error_m": sim.errors[-1][0] if sim.errors else 0,
                               "geoms": [{"p": p.tolist(), "r": r.tolist()}
                                         for p, r in zip(sim.data.geom_xpos, sim.data.geom_xmat)]}
                return self.reply(json.dumps(payload, allow_nan=False).encode())
            self.reply(b"{}", code=404)

        def do_POST(self):
            if self.path != "/control" or self.headers.get("Origin", f"http://127.0.0.1:{port}") not in (
                f"http://127.0.0.1:{port}", f"http://localhost:{port}"
            ):
                return self.reply(b"{}", code=403)
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 128:
                    raise ValueError("invalid request size")
                action = json.loads(self.rfile.read(size))["action"]
                with lock:
                    if action == "pause":
                        state["paused"] = True
                    elif action == "resume":
                        state["paused"] = False
                    elif action == "reset":
                        from .official_simulation import Simulation
                        sim.mujoco.mj_resetDataKeyframe(sim.model, sim.data, 0)
                        sim.mujoco.mj_forward(sim.model, sim.data)
                        replacement = Simulation(sim.model, sim.data, sim.solver, sim.config)
                        sim.__dict__.update(replacement.__dict__)
                    else:
                        raise ValueError("unknown simulation action")
                return self.reply(b'{"ok":true}')
            except (ValueError, KeyError, TypeError):
                return self.reply(b"{}", code=400)

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = f"http://127.0.0.1:{port}/"
    print("OFFICIAL_IK_SIMULATION_URL=" + url, flush=True)
    if open_browser:
        webbrowser.open(url)
    next_status = 0.0
    try:
        while not stopping.is_set():
            started = time.perf_counter()
            with lock:
                if not state["paused"]:
                    sim.advance(sim.config.period_s)
                if sim.data.time >= next_status:
                    status_path.write_text(json.dumps(sim.summary(), indent=2), encoding="utf-8")
                    next_status = sim.data.time + 1.0
            stopping.wait(max(0.0, sim.config.period_s - (time.perf_counter() - started)))
    except KeyboardInterrupt:
        pass
    finally:
        stopping.set()
        server.shutdown()
        server.server_close()
