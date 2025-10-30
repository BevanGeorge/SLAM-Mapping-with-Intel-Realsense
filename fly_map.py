import os, math, time, threading
import pybullet as p
import pybullet_data as pd

# ===== USER SETTINGS =====
MESH_PATH = "/home/bevan/Desktop/drone_sim/map.obj"  # absolute path
TARGET_MAX_SIZE = 60.0     # autoscale largest side to ~this (m)
BASE_SPEED = 3.0           # m/s
START_CAM_MODE = 1         # 0=orbit, 1=chase, 2=fpv
SAVE_FRAMES = False        # True => writes ./frames/rgb_*.png + depth_*.npy
FRAME_W, FRAME_H = 960, 540
# =========================

def quat_from_yaw(yaw): return p.getQuaternionFromEuler([0,0,yaw])

def key_is_down(keys, codes):
    for c in codes:
        if c in keys and (keys[c] & p.KEY_IS_DOWN or keys[c] & p.KEY_WAS_TRIGGERED):
            return True
    return False

# ---- init
cid = p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
for opt,val in [(getattr(p,"COV_ENABLE_GUI",None),0),
                (getattr(p,"COV_ENABLE_WIREFRAME",None),0),
                (getattr(p,"COV_ENABLE_SHADOWS",None),1)]:
    if opt is not None:
        try: p.configureDebugVisualizer(opt,val)
        except: pass
p.setGravity(0,0,-9.81)

if not os.path.isfile(MESH_PATH):
    raise FileNotFoundError(MESH_PATH)

# ---- load mesh (measure -> scale -> real body)
vis_tmp = p.createVisualShape(p.GEOM_MESH, fileName=MESH_PATH, meshScale=[1,1,1])
col_tmp = p.createCollisionShape(p.GEOM_MESH, fileName=MESH_PATH, meshScale=[1,1,1],
                                 flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
tmp = p.createMultiBody(0, col_tmp, vis_tmp, [0,0,0])
a0,a1 = p.getAABB(tmp); p.removeBody(tmp)

size = [a1[i]-a0[i] for i in range(3)]
scale = TARGET_MAX_SIZE / max(max(size),1e-6)
meshScale = [scale]*3

visual = p.createVisualShape(p.GEOM_MESH, fileName=MESH_PATH, meshScale=meshScale,
                             rgbaColor=[0.80,0.82,0.88,1])
collision = p.createCollisionShape(p.GEOM_MESH, fileName=MESH_PATH, meshScale=meshScale,
                                   flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
world = p.createMultiBody(0, collision, visual, [0,0,0])

a0,a1 = p.getAABB(world)
center = [(a0[i]+a1[i])/2 for i in range(3)]
extent = [a1[i]-a0[i] for i in range(3)]
spawn_z = a1[2] + 0.5

drone = p.loadURDF("r2d2.urdf", [center[0],center[1],spawn_z], quat_from_yaw(0))

def cam_orbit():
    dist = max(12.0, 1.3*max(extent[0], extent[1]))
    p.resetDebugVisualizerCamera(dist, 45, -30, center)
def cam_chase(pos,yaw):
    p.resetDebugVisualizerCamera(6.0, math.degrees(-yaw)+90.0, -20.0, pos)
def cam_fpv(pos,yaw):
    p.resetDebugVisualizerCamera(1.2, math.degrees(-yaw), 0.0, pos)

cam_mode = START_CAM_MODE
cam_orbit() if cam_mode==0 else cam_chase([center[0],center[1],spawn_z],0.0) if cam_mode==1 else cam_fpv([center[0],center[1],spawn_z],0.0)

print("""
CLICK ON THE 3D WINDOW FIRST.
Controls:
  W/S = forward/back    |  A/D = strafe left/right
  ← / → = yaw           |  Q/E = up/down
  F or 1/2/3 = camera   |  +/- or =/_ = speed up/down
  R = reset             |  G = toggle gravity
  SPACE = stop motion
""")

# ----- optional RGB-D
if SAVE_FRAMES:
    import numpy as np, imageio, pathlib
    pathlib.Path("frames").mkdir(exist_ok=True)
def render_and_save(pos,yaw,idx):
    if not SAVE_FRAMES: return
    import numpy as np, imageio
    cy, sy = math.cos(yaw), math.sin(yaw)
    cam_off = [-2.0, 0.0, 0.8]
    cam = [ pos[0] + cy*cam_off[0] + sy*cam_off[1],
            pos[1] - sy*cam_off[0] + cy*cam_off[1],
            pos[2] + cam_off[2] ]
    tgt = [ pos[0] + cy*5.0, pos[1] - sy*5.0, pos[2] ]
    view = p.computeViewMatrix(cam, tgt, [0,0,1])
    proj = p.computeProjectionMatrixFOV(70.0, FRAME_W/FRAME_H, 0.1, 200.0)
    w,h,rgba,depth,seg = p.getCameraImage(FRAME_W,FRAME_H,view,proj,renderer=p.ER_BULLET_HARDWARE_OPENGL)[:5]
    imageio.imwrite(f"frames/rgb_{idx:05d}.png", (rgba[...,:3]).astype('uint8'))
    import numpy as np; np.save(f"frames/depth_{idx:05d}.npy", np.array(depth, dtype=np.float32))

# ----- input loop
keys = {}
def kb_loop():
    global keys
    while p.isConnected():
        keys = p.getKeyboardEvents(); time.sleep(0.03)
threading.Thread(target=kb_loop, daemon=True).start()

yaw, speed, use_gravity = 0.0, BASE_SPEED, False
dt, frame_idx = 1/120, 0

while p.isConnected():
    # gather once per frame
    k = dict(keys)

    # camera hotkeys
    if key_is_down(k, [ord('1')]): cam_mode=0; cam_orbit()
    if key_is_down(k, [ord('2')]): cam_mode=1
    if key_is_down(k, [ord('3')]): cam_mode=2
    if key_is_down(k, [ord('F'), ord('f')]):
        cam_mode = (cam_mode+1)%3

    # speed hotkeys (covers +/= and -/_ keys)
    if key_is_down(k, [ord('+'), ord('=' )]): speed *= 1.25
    if key_is_down(k, [ord('-'), ord('_')]): speed /= 1.25

    # reset / gravity
    if key_is_down(k, [ord('R'), ord('r')]):
        yaw = 0.0
        p.resetBasePositionAndOrientation(drone, [center[0],center[1],spawn_z], quat_from_yaw(yaw))
    if key_is_down(k, [ord('G'), ord('g')]):
        use_gravity = not use_gravity
        p.setGravity(0,0,-9.81 if use_gravity else 0.0)

    # yaw
    if key_is_down(k, [p.B3G_LEFT_ARROW]):  yaw += 1.8*dt
    if key_is_down(k, [p.B3G_RIGHT_ARROW]): yaw -= 1.8*dt

    # motion (relative to yaw)
    fwd = 1.0 if key_is_down(k,[ord('W'),ord('w')]) else -1.0 if key_is_down(k,[ord('S'),ord('s')]) else 0.0
    strf = 1.0 if key_is_down(k,[ord('D'),ord('d')]) else -1.0 if key_is_down(k,[ord('A'),ord('a')]) else 0.0
    up   = 1.0 if key_is_down(k,[ord('Q'),ord('q')]) else -1.0 if key_is_down(k,[ord('E'),ord('e')]) else 0.0
    if key_is_down(k,[32]): fwd=strf=up=0.0   # SPACE

    pos,_ = p.getBasePositionAndOrientation(drone)
    cy, sy = math.cos(yaw), math.sin(yaw)
    dx = ( cy*fwd +  sy*strf) * speed * dt
    dy = (-sy*fwd +  cy*strf) * speed * dt
    dz = up * speed * dt
    newpos = [pos[0]+dx, pos[1]+dy, max(a0[2]+0.05, pos[2]+dz)]
    p.resetBasePositionAndOrientation(drone, newpos, quat_from_yaw(yaw))

    # camera follow
    if cam_mode==0: cam_orbit()
    elif cam_mode==1: cam_chase(newpos, yaw)
    else: cam_fpv(newpos, yaw)

    render_and_save(newpos, yaw, frame_idx); frame_idx += 1
    p.stepSimulation(); time.sleep(dt)

