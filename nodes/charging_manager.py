#!/usr/bin/env python3
"""
charging_manager.py — palmares_bot: bateria, fila de tarefas e dock como home.

Ciclo:
  STARTUP (30 s) → RETURNING_HOME → DOCKING → HOME_CHARGING
  Ocioso (fila vazia ou bat < 40%) → permanece em HOME_CHARGING (dock)
  Tarefa na fila + bat ≥ 40 % → UNDOCKING → GOING_TO_TASK → EXECUTING_TASK
                               → RETURNING_HOME → DOCKING → HOME_CHARGING
  bat < 20 % após tarefa → retorna imediatamente
  bat ≤  5 % durante tarefa → emergência, aborta e retorna

Retorno ao dock:
  _start_returning_home → DockRobot(navigate_to_staging_pose=True)
  O servidor de docking navega até o staging e faz a abordagem precisa.

Tópico manual: publique em /go_charge (std_msgs/Empty) para forçar retorno.

Bateria:
  Drena  0.5 %/s fora de HOME_CHARGING →  100→0 em 200 s
  Carrega 1.0 %/s em HOME_CHARGING      →    0→100 em 100 s
"""

import math
import rclpy
import rclpy.time
from collections import deque
from dataclasses import dataclass
from typing import Optional, Deque, List

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Float32, Empty
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose, DockRobot, UndockRobot


# ── Estados ────────────────────────────────────────────────────────────────────
STARTUP               = 'STARTUP'
NAVIGATING_TO_STAGING = 'NAVIGATING_TO_STAGING'
CENTERING             = 'CENTERING'
DOCKING               = 'DOCKING'
HOME_CHARGING         = 'HOME_CHARGING'
UNDOCKING             = 'UNDOCKING'
GOING_TO_TASK         = 'GOING_TO_TASK'
EXECUTING_TASK        = 'EXECUTING_TASK'
RETURNING_HOME        = 'RETURNING_HOME'

# ── Bateria ────────────────────────────────────────────────────────────────────
BATT_DRAIN    = 0.5   # %/s fora de casa  → 100→0 em 200 s
BATT_CHARGE   = 1.0   # %/s em HOME_CHARGING → 0→100 em 100 s
MIN_FOR_TASK  = 40.0  # % mínimo para aceitar tarefa da fila
LOW_AFTER     = 20.0  # % → forçar retorno após tarefa
EMERGENCY     = 5.0   # % → abortar tarefa imediatamente

# ── Dock / Staging ─────────────────────────────────────────────────────────────
# Galpão de fábrica (warehouse 30×50 m). Spawn world(0,0) = odom(0,0).
# Dock world(13,0) = odom(13,0), parede direita do galpão.
# Staging odom(12,0) — 1 m à frente, calculado pelo docking server.
STAGING_X        = 12.0
STAGING_Y        = 0.0
STAGING_YAW      = 0.0
DOCK_MAX_FAILS   = 4    # retries antes de pausa longa (nunca força HOME sem estar lá)

# Verificação física de docking — HOME_CHARGING SOMENTE quando o robô está
# fisicamente próximo do dock. Resolve o bug onde spawn em (0,0,yaw=0)
# passava na checagem de Y+yaw e entrava em HOME_CHARGING longe do dock.
# Dock em odom(13,0). Robot para ~30cm à frente da face: odom X ≈ 12.9.
DOCK_X_ODOM    = 12.7  # X esperado do robô dockado (odom)
DOCK_Y_ODOM    = 0.0   # Y esperado do robô dockado (odom)
ALIGN_TOL_XY   = 2.0   # m   — raio 2D: bloqueia spawn em X=0 (dist=12.7m), aceita variações de docking
ALIGN_TOL_YAW  = 1.0   # rad — ≈ 57° — permissivo, evita falso-negativo por yaw residual
DOCK_LOST_DIST = 1.0   # m   — se HOME_CHARGING e robô > 1m do dock → foi deslocado, retornar

CTR_SAMPLE_S     = 2.0
CTR_DEAD_BAND    = 0.05
CTR_MAX_CORR     = 0.30

# Reverse para undocking simples (sem UndockRobot action)
UNDOCK_SPEED     = -0.15   # m/s
UNDOCK_DURATION  = 2.0     # s  →  recua ~30 cm

# ── Rotas de patrulha (x, y, yaw_rad em odom) ────────────────────────────────
# Galpão de fábrica. odom = world (spawn em world origin).
# Área de trabalho: central/esquerda do galpão (odom x < 10).
# Dock em odom(13,0) — rotas ficam distantes do dock.
PATROL_ROUTES: dict = {
    'A': [(3.0, 7.0, 0.0),   (-4.0, 7.0,  1.57),  (-4.0, -7.0, 3.14)],
    'B': [(-2.0, 3.0, 0.0),  (-7.0, 0.0,  0.0),   (-2.0, -3.0, 0.0) ],
    'C': [(5.0, -5.0, 0.0),  (-3.0, -5.0, 1.57),  (-3.0,  5.0, 0.0) ],
}

TASK_WAIT_S = {'goto_pose': 2, 'inspect': 5, 'deliver': 3, 'patrol': 0}

AUTOSTART_DELAY = 30.0  # s após startup (Nav2 t=20s, docking server t=25s)


@dataclass
class Task:
    type:  str
    pose:  Optional[PoseStamped] = None
    route: str = ''


def _make_pose(x: float, y: float, yaw: float) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id    = 'odom'
    ps.pose.position.x    = x
    ps.pose.position.y    = y
    ps.pose.orientation.z = math.sin(yaw / 2.0)
    ps.pose.orientation.w = math.cos(yaw / 2.0)
    return ps


def _nav_goal(pose: PoseStamped) -> NavigateToPose.Goal:
    g = NavigateToPose.Goal()
    g.pose = pose
    g.pose.header.stamp = rclpy.time.Time().to_msg()
    return g


class ChargingManager(Node):

    def __init__(self):
        super().__init__('charging_manager')

        self._state             = STARTUP
        self._battery           = 100.0
        self._tick_count        = 0
        self._queue: Deque[Task]     = deque()
        self._task: Optional[Task]   = None
        self._patrol_wps: List[PoseStamped] = []
        self._patrol_idx        = 0
        self._dock_retries      = 0
        self._centering_samples: list = []
        self._centering_timer   = None
        self._centering_sub     = None
        self._robot_x: Optional[float]   = None
        self._robot_y: Optional[float]   = None
        self._robot_yaw: Optional[float] = None

        cb = ReentrantCallbackGroup()

        self.create_subscription(Odometry,    '/odom',           self._odom_cb,      10, callback_group=cb)
        self.create_subscription(PoseStamped, '/task/goto_pose', self._cb_goto,      10, callback_group=cb)
        self.create_subscription(PoseStamped, '/task/inspect',   self._cb_inspect,   10, callback_group=cb)
        self.create_subscription(String,      '/task/patrol',    self._cb_patrol,    10, callback_group=cb)
        self.create_subscription(PoseStamped, '/task/deliver',   self._cb_deliver,   10, callback_group=cb)
        self.create_subscription(Empty,       '/go_charge',      self._cb_go_charge, 10, callback_group=cb)

        self._state_pub   = self.create_publisher(String,  '/charging_manager/state', 10)
        self._battery_pub = self.create_publisher(Float32, '/battery_level', 10)
        self._vel_pub     = self.create_publisher(Twist,   '/cmd_vel', 10)

        self._nav    = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=cb)
        self._dockc  = ActionClient(self, DockRobot,      'dock_robot',       callback_group=cb)
        self._undockc = ActionClient(self, UndockRobot,   'undock_robot',     callback_group=cb)

        self.create_timer(1.0, self._battery_tick)
        self._once(AUTOSTART_DELAY, self._autostart)
        self.get_logger().info(
            f'ChargingManager pronto. Vai para dock em {AUTOSTART_DELAY:.0f} s.')

    # ── Helpers ────────────────────────────────────────────────────────────────
    def _once(self, delay: float, fn):
        h = [None]
        def _cb(): h[0].cancel(); fn()
        h[0] = self.create_timer(delay, _cb)

    def _set_state(self, s: str):
        self._state = s
        m = String(); m.data = s
        self._state_pub.publish(m)
        self.get_logger().info(f'→ {s}  (bat={self._battery:.1f}%)')

    def _odom_cb(self, msg: Odometry):
        self._robot_x   = msg.pose.pose.position.x
        self._robot_y   = msg.pose.pose.position.y
        qz, qw = msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        self._robot_yaw = 2.0 * math.atan2(qz, qw)

    # ── Bateria ────────────────────────────────────────────────────────────────
    def _battery_tick(self):
        self._tick_count += 1

        if self._state == HOME_CHARGING:
            self._battery = min(100.0, self._battery + BATT_CHARGE)
            if self._robot_x is not None:
                dist = math.sqrt(
                    (self._robot_x - DOCK_X_ODOM) ** 2 +
                    (self._robot_y - DOCK_Y_ODOM) ** 2)
                if dist > DOCK_LOST_DIST:
                    self.get_logger().warn(
                        f'HOME_CHARGING mas robô deslocado ({dist:.2f}m do dock) — retornando.')
                    self._start_returning_home()
        elif self._state != STARTUP:
            self._battery = max(EMERGENCY, self._battery - BATT_DRAIN)

        m = Float32(); m.data = float(self._battery)
        self._battery_pub.publish(m)

        # Republica estado a cada tick para que 'ros2 topic echo --once' funcione
        s = String(); s.data = self._state
        self._state_pub.publish(s)

        # Log a cada 5 s
        if self._tick_count % 5 == 0:
            pct = self._battery
            bar = '█' * int(pct // 10) + '░' * (10 - int(pct // 10))
            self.get_logger().info(
                f'[bat {bar} {pct:5.1f}%]  {self._state}  fila={len(self._queue)}')

        # Emergência durante tarefa
        if (self._battery <= EMERGENCY and
                self._state in (GOING_TO_TASK, EXECUTING_TASK)):
            self.get_logger().error(
                f'BATERIA CRÍTICA {self._battery:.1f}% — abortando tarefa!')
            self._task = None
            self._start_returning_home()
            return

        # Processar fila quando em HOME_CHARGING com bateria suficiente
        if self._state == HOME_CHARGING and self._battery >= MIN_FOR_TASK:
            self._check_queue()

    # ── Fila de tarefas ────────────────────────────────────────────────────────
    def _enqueue(self, task: Task):
        self._queue.append(task)
        self.get_logger().info(
            f'[FILA +{task.type}]  total={len(self._queue)}  bat={self._battery:.1f}%  '
            f'estado={self._state}')

    def _check_queue(self):
        # _task is None garante que não fazemos pop duplo enquanto _start_undocking retenta
        if (self._queue and self._task is None
                and self._state == HOME_CHARGING
                and self._battery >= MIN_FOR_TASK):
            self._task = self._queue.popleft()
            self.get_logger().info(
                f'[FILA] Aceitando {self._task.type}  bat={self._battery:.1f}%')
            self._start_undocking()

    def _cb_goto(self,    m: PoseStamped): self._enqueue(Task('goto_pose', pose=m))
    def _cb_inspect(self, m: PoseStamped): self._enqueue(Task('inspect',   pose=m))
    def _cb_deliver(self, m: PoseStamped): self._enqueue(Task('deliver',   pose=m))
    def _cb_patrol(self,  m: String):
        r = m.data.strip().upper()
        if r not in PATROL_ROUTES:
            self.get_logger().warn(
                f'Rota desconhecida: {r!r}  opções={list(PATROL_ROUTES)}')
            return
        self._enqueue(Task('patrol', route=r))

    def _cb_go_charge(self, _: Empty):
        if self._state in (HOME_CHARGING, DOCKING, RETURNING_HOME):
            self.get_logger().info('/go_charge recebido — já em retorno/carga, ignorando.')
            return
        self.get_logger().info('/go_charge recebido — abortando tarefa e retornando ao dock.')
        self._task = None
        self._start_returning_home()

    # ── STARTUP ────────────────────────────────────────────────────────────────
    def _autostart(self):
        if self._state == STARTUP:
            self.get_logger().info(
                'Startup: navegando até a base de carregamento (factory world).')
            self._start_returning_home()

    # ── NAVIGATING_TO_STAGING ─────────────────────────────────────────────────
    def _navigate_to_staging(self):
        # Não muda estado antes de confirmar servidor — evita drain de bateria enquanto Nav2 sobe
        if not self._nav.server_is_ready():
            self.get_logger().warn('Nav2 não disponível — aguardando 3 s')
            self._once(3.0, self._navigate_to_staging)
            return
        self._set_state(NAVIGATING_TO_STAGING)
        self._nav.send_goal_async(
            _nav_goal(_make_pose(STAGING_X, STAGING_Y, STAGING_YAW))
        ).add_done_callback(self._staging_acc)

    def _staging_acc(self, f):
        h = f.result()
        if not h.accepted:
            self._once(2.0, self._navigate_to_staging); return
        h.get_result_async().add_done_callback(self._staging_res)

    def _staging_res(self, f):
        if f.result().status != 4:
            self.get_logger().warn('Staging falhou — retry 3 s')
            self._once(3.0, self._navigate_to_staging); return
        self._start_centering()

    # ── CENTERING ─────────────────────────────────────────────────────────────
    def _start_centering(self):
        self._set_state(CENTERING)
        self._centering_samples = []
        self._centering_sub = self.create_subscription(
            PoseStamped, '/detected_dock_pose', self._ctr_pose_cb, 10)
        self._centering_timer = self.create_timer(CTR_SAMPLE_S, self._centering_done)

    def _ctr_pose_cb(self, msg: PoseStamped):
        if self._state == CENTERING:
            self._centering_samples.append(msg.pose.position.y)

    def _centering_done(self):
        self._centering_timer.cancel(); self._centering_timer = None
        if self._centering_sub:
            self.destroy_subscription(self._centering_sub)
            self._centering_sub = None
        if not self._centering_samples:
            self.get_logger().warn('Sem amostras — pulando centering.')
            self._start_docking(); return
        err = sum(self._centering_samples) / len(self._centering_samples) - STAGING_Y
        if abs(err) < CTR_DEAD_BAND:
            self._start_docking(); return
        cy = STAGING_Y + max(-CTR_MAX_CORR, min(CTR_MAX_CORR, err))
        self._nav.send_goal_async(
            _nav_goal(_make_pose(STAGING_X, cy, STAGING_YAW))
        ).add_done_callback(self._ctr_nav_cb)

    def _ctr_nav_cb(self, f):
        h = f.result()
        if not h.accepted:
            self._start_docking(); return
        h.get_result_async().add_done_callback(lambda _: self._start_docking())

    # ── DOCKING ───────────────────────────────────────────────────────────────
    def _start_docking(self):
        if not self._dockc.server_is_ready():
            self.get_logger().warn('DockRobot não disponível — aguardando 2 s')
            self._once(2.0, self._start_docking)
            return
        self._set_state(DOCKING)
        g = DockRobot.Goal()
        g.use_dock_id = True
        g.dock_id = 'base_carregamento'
        g.navigate_to_staging_pose = True   # DockRobot navega até staging e faz abordagem com LiDAR/ArUco
        self._dockc.send_goal_async(g).add_done_callback(self._dock_acc)

    def _dock_acc(self, f):
        if self._state not in (DOCKING, RETURNING_HOME):
            return
        h = f.result()
        if not h.accepted:
            self._once(2.0, self._start_docking); return
        h.get_result_async().add_done_callback(self._dock_res)

    def _dock_res(self, f):
        if self._state not in (DOCKING, RETURNING_HOME):
            self.get_logger().warn(
                f'_dock_res ignorado — estado atual: {self._state}')
            return

        # Posição atual para log e proteção contra HOME_CHARGING longe do dock
        rx   = self._robot_x or 0.0
        ry   = self._robot_y or 0.0
        dist = math.sqrt((rx - DOCK_X_ODOM)**2 + (ry - DOCK_Y_ODOM)**2)

        if f.result().status != 4:
            self._dock_retries += 1
            self.get_logger().warn(
                f'DockRobot falhou ({self._dock_retries}/{DOCK_MAX_FAILS}) '
                f'dist={dist:.2f}m pos=({rx:.1f},{ry:.1f})')
            if self._dock_retries >= DOCK_MAX_FAILS:
                self._dock_retries = 0
                if dist <= ALIGN_TOL_XY:
                    self.get_logger().warn(
                        f'Máx retries — robô perto do dock (dist={dist:.2f}m) → HOME_CHARGING.')
                    self._set_state(HOME_CHARGING)
                else:
                    self.get_logger().error(
                        f'Máx retries — robô longe (dist={dist:.2f}m) → retry em 15 s.')
                    self._once(15.0, self._start_docking)
            else:
                self._once(3.0, self._start_docking)
            return

        # DockRobot status=4 — verificar alinhamento físico
        yaw_err = abs(self._robot_yaw or 0.0)
        aligned = dist <= ALIGN_TOL_XY and yaw_err <= ALIGN_TOL_YAW

        if not aligned:
            self._dock_retries += 1
            self.get_logger().warn(
                f'Fora de alinhamento após docking  dist={dist:.2f}m '
                f'yaw={math.degrees(yaw_err):.1f}°  pos=({rx:.1f},{ry:.1f})  '
                f'({self._dock_retries}/{DOCK_MAX_FAILS})')
            if self._dock_retries >= DOCK_MAX_FAILS:
                self.get_logger().warn('Máx realinhamentos — forçando HOME_CHARGING.')
                self._dock_retries = 0
                self._set_state(HOME_CHARGING)
            else:
                self._once(3.0, self._start_docking)
            return

        self._dock_retries = 0
        self.get_logger().info(
            f'Dockado ✓  dist={dist:.2f}m  yaw={math.degrees(yaw_err):.1f}°  '
            f'pos=({rx:.2f},{ry:.2f})  bat={self._battery:.1f}%')
        self._set_state(HOME_CHARGING)

    # ── UNDOCKING → tarefa (reverse simples, sem ação ROS) ───────────────────
    def _start_undocking(self):
        self._set_state(UNDOCKING)
        self.get_logger().info(
            f'Recuando da dock ({abs(UNDOCK_SPEED):.2f} m/s × {UNDOCK_DURATION:.1f} s)…')
        cmd = Twist()
        cmd.linear.x = UNDOCK_SPEED
        self._vel_pub.publish(cmd)
        self._once(UNDOCK_DURATION, self._undock_done)

    def _undock_done(self):
        if self._state != UNDOCKING:
            self._vel_pub.publish(Twist()); return
        self._vel_pub.publish(Twist())
        self.get_logger().info('Undock completo — indo para tarefa.')
        self._start_going_to_task()

    # ── GOING_TO_TASK ─────────────────────────────────────────────────────────
    def _start_going_to_task(self):
        if self._task is None:
            self._start_returning_home(); return
        if not self._nav.server_is_ready():
            self.get_logger().warn('Nav2 não disponível — aguardando 3 s para tarefa')
            self._once(3.0, self._start_going_to_task); return
        self._set_state(GOING_TO_TASK)
        t = self._task
        if t.type == 'patrol':
            self._patrol_wps = [_make_pose(*wp) for wp in PATROL_ROUTES[t.route]]
            self._patrol_idx = 0
            self._next_patrol_wp()
        elif t.pose:
            self._nav.send_goal_async(
                _nav_goal(t.pose)
            ).add_done_callback(self._task_nav_acc)
        else:
            self._start_executing_task()

    def _task_nav_acc(self, f):
        if self._state != GOING_TO_TASK:
            return
        h = f.result()
        if not h.accepted:
            self._once(2.0, self._start_going_to_task); return
        h.get_result_async().add_done_callback(self._task_nav_res)

    def _task_nav_res(self, f):
        if self._state != GOING_TO_TASK:
            return  # emergência ou go_charge interrompeu antes do callback
        if f.result().status != 4:
            self.get_logger().warn('Nav até tarefa falhou — retornando')
            self._task = None; self._start_returning_home(); return
        self._start_executing_task()

    # ── PATROL ────────────────────────────────────────────────────────────────
    def _next_patrol_wp(self):
        if self._patrol_idx >= len(self._patrol_wps):
            self.get_logger().info('Patrulha concluída.')
            self._start_executing_task(); return
        wp = self._patrol_wps[self._patrol_idx]
        self._patrol_idx += 1
        self.get_logger().info(
            f'Patrol wp {self._patrol_idx}/{len(self._patrol_wps)}')
        self._nav.send_goal_async(
            _nav_goal(wp)
        ).add_done_callback(self._patrol_wp_acc)

    def _patrol_wp_acc(self, f):
        if self._state != GOING_TO_TASK:
            return
        h = f.result()
        if not h.accepted:
            self._next_patrol_wp(); return
        h.get_result_async().add_done_callback(self._patrol_wp_res)

    def _patrol_wp_res(self, f):
        if self._state != GOING_TO_TASK:
            return  # emergência ou go_charge interrompeu durante patrulha
        if f.result().status != 4:
            self.get_logger().warn('Waypoint falhou — abortando patrulha')
            self._finish_task(); return
        self._next_patrol_wp()

    # ── EXECUTING_TASK ────────────────────────────────────────────────────────
    def _start_executing_task(self):
        self._set_state(EXECUTING_TASK)
        t = self._task
        wait = TASK_WAIT_S.get(t.type if t else '', 0)
        self.get_logger().info(
            f'Executando {t.type if t else "?"} — aguardando {wait} s')
        if wait > 0:
            self._once(float(wait), self._finish_task)
        else:
            self._finish_task()

    def _finish_task(self):
        if self._state != EXECUTING_TASK:
            return
        t = self._task; self._task = None
        self.get_logger().info(
            f'Tarefa {t.type if t else "?"} concluída.  '
            f'bat={self._battery:.1f}%  fila={len(self._queue)}')

        if self._battery < LOW_AFTER:
            self.get_logger().warn(
                f'Bateria baixa ({self._battery:.1f}%) — forçando retorno.')
            self._start_returning_home(); return

        if self._queue and self._battery >= MIN_FOR_TASK:
            self._task = self._queue.popleft()
            self.get_logger().info(
                f'Próxima tarefa direto: {self._task.type}')
            self._start_going_to_task(); return

        self._start_returning_home()

    # ── RETURNING_HOME ────────────────────────────────────────────────────────
    def _start_returning_home(self):
        self._set_state(RETURNING_HOME)
        self._dock_retries = 0
        # DockRobot navega até staging e faz abordagem com dock_pose_estimator (ArUco + LiDAR)
        self._start_docking()


def main(args=None):
    rclpy.init(args=args)
    node = ChargingManager()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
