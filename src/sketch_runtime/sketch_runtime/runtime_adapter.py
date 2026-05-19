from typing import List, Optional

try:
    from kinematics_msgs.srv import SetRobotPose as IKSetRobotPose
    _HAS_IK_SRV = True
except Exception:
    IKSetRobotPose = None
    _HAS_IK_SRV = False


class RuntimeAdapter:
    def __init__(self, node=None, dry_run: bool = True,
                 enable_real_ik: bool = False,
                 enable_real_servo: bool = False):
        self._node = node
        self.dry_run = dry_run
        self.enable_real_ik = enable_real_ik
        self.enable_real_servo = enable_real_servo
        self._log_fn = None
        if node is not None:
            self._log_fn = node.get_logger().info
        self._call_log: List[dict] = []

        self._ik_cli = None
        self._servo_pub = None
        if node is not None and not dry_run and enable_real_ik and _HAS_IK_SRV:
            self._ik_cli = node.create_client(
                IKSetRobotPose, "/kinematics/set_pose_target"
            )

    def _log(self, msg: str):
        if self._log_fn:
            self._log_fn(msg)
        else:
            print(f"[RuntimeAdapter] {msg}")

    def _record(self, method: str, args: dict, result: any = None, error: str = None):
        entry = {"method": method, "args": args}
        if result is not None:
            entry["result"] = result
        if error:
            entry["error"] = error
        self._call_log.append(entry)

    def mode_summary(self) -> str:
        if self.dry_run:
            return "DRY_RUN (ik=mock servo=mock)"
        parts = []
        parts.append(f"ik={'REAL' if self.enable_real_ik else 'OFF'}")
        parts.append(f"servo={'REAL' if self.enable_real_servo else 'OFF'}")
        return "LIVE: " + " ".join(parts)

    async def ik_solve(
        self,
        position: List[float],
        rpy: Optional[List[float]] = None,
        pitch: float = 80.0,
        pitch_range: List[float] = None,
        resolution: float = 1.0,
        timeout_sec: float = 8.0,
    ) -> Optional[List[int]]:
        self._log(
            f"ik_solve position={position} rpy={rpy} pitch={pitch} "
            f"resolution={resolution} mode={self.mode_summary()}"
        )

        if self.dry_run:
            result = [500, 500, 500, 500, 500]
            self._record("ik_solve", {
                "position": position, "rpy": rpy or [0, 0, 0],
                "pitch": pitch, "resolution": resolution,
            }, result)
            return result

        if not self.enable_real_ik:
            msg = "real execution requested but enable_real_ik is false"
            self._log(f"FAIL: {msg}")
            self._record("ik_solve", {
                "position": position, "rpy": rpy or [0, 0, 0],
            }, error=msg)
            return None

        if self._ik_cli is None:
            msg = "IK client not created (kinematics_msgs unavailable or node is None)"
            self._log(f"FAIL: {msg}")
            self._record("ik_solve", {
                "position": position, "rpy": rpy or [0, 0, 0],
            }, error=msg)
            return None

        try:
            req = IKSetRobotPose.Request()
            req.position = [float(p) for p in position[:3]]
            req.pitch = float(pitch)
            req.pitch_range = list(pitch_range or [-180.0, 180.0])
            req.resolution = float(resolution)

            future = self._ik_cli.call_async(req)
            import time as _time
            deadline = _time.time() + timeout_sec
            import rclpy
            while rclpy.ok() and _time.time() < deadline:
                if future.done():
                    res = future.result()
                    if res is not None and res.success:
                        pulses = []
                        for attr in ("pulse", "pulses", "positions"):
                            val = getattr(res, attr, None)
                            if val:
                                pulses = [int(p) for p in list(val)]
                                break
                        if pulses:
                            self._record("ik_solve", {
                                "position": position, "rpy": rpy or [0, 0, 0],
                                "pitch": pitch, "resolution": resolution,
                            }, pulses)
                            self._log(f"ik_solve OK → pulses={pulses}")
                            return pulses
                    msg = f"IK service returned success=false or None"
                    self._log(f"FAIL: {msg}")
                    self._record("ik_solve", {
                        "position": position, "rpy": rpy or [0, 0, 0],
                    }, error=msg)
                    return None
                rclpy.spin_once(self._node, timeout_sec=0.02)
            msg = f"IK timeout after {timeout_sec}s"
            self._log(f"FAIL: {msg}")
            self._record("ik_solve", {
                "position": position, "rpy": rpy or [0, 0, 0],
            }, error=msg)
            return None
        except Exception as e:
            msg = f"IK exception: {e}"
            self._log(f"FAIL: {msg}")
            self._record("ik_solve", {
                "position": position, "rpy": rpy or [0, 0, 0],
            }, error=msg)
            return None

    async def servo_move(self, pulses: List[int], duration_ms: int = 2000):
        self._log(
            f"servo_move pulses={pulses} duration_ms={duration_ms} "
            f"mode={self.mode_summary()}"
        )
        self._record("servo_move", {
            "pulses": pulses, "duration_ms": duration_ms,
        })
        if self.dry_run or not self.enable_real_servo:
            return

        if self._servo_pub is None and self._node is not None:
            try:
                from servo_controller_msgs.msg import ServosPosition, ServoPosition
                self._servo_pub = self._node.create_publisher(
                    ServosPosition, "/servo_controller", 10
                )
            except Exception:
                pass

        if self._servo_pub is None:
            self._log("WARN: servo publisher not available, real servo move skipped")
            return

        try:
            from servo_controller_msgs.msg import ServosPosition, ServoPosition
            msg = ServosPosition()
            msg.duration = float(duration_ms) / 1000.0
            msg.position_unit = "pulse"
            for i, p in enumerate(pulses[:5], start=1):
                sp = ServoPosition()
                sp.id = i
                sp.position = max(0, min(1000, int(p)))
                msg.position.append(sp)
            self._servo_pub.publish(msg)
        except Exception as e:
            self._log(f"servo_move publish failed: {e}")

    async def gripper_set(self, servo_id: int, pulse: int, duration_ms: int = 300):
        self._log(
            f"gripper_set servo_id={servo_id} pulse={pulse} "
            f"duration_ms={duration_ms} mode={self.mode_summary()}"
        )
        self._record("gripper_set", {
            "servo_id": servo_id, "pulse": pulse, "duration_ms": duration_ms,
        })
        if self.dry_run or not self.enable_real_servo:
            return

        await self.servo_move([pulse], duration_ms)

    async def get_joint_state(self):
        self._log(f"get_joint_state mode={self.mode_summary()}")
        if self.dry_run:
            return None
        return None

    async def query_vision(self, class_name: str = "", timeout_sec: float = 2.0):
        self._log(
            f"query_vision class_name={class_name} "
            f"timeout_sec={timeout_sec} mode={self.mode_summary()}"
        )
        if self.dry_run:
            return None
        return None
