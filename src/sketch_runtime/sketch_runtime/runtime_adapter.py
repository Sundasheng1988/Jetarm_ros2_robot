from typing import List, Optional
import asyncio
import time as _time

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

        self._servo_pub = None

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
        if pitch_range is None:
            pitch_range = [-90.0, 90.0]

        self._log(
            f"ik_solve position={position} rpy={rpy} pitch={pitch} "
            f"pitch_range={pitch_range} resolution={resolution} "
            f"mode={self.mode_summary()}"
        )

        if self.dry_run:
            result = [500, 500, 500, 500, 500]
            self._record("ik_solve", {
                "position": position, "rpy": rpy or [0, 0, 0],
                "pitch": pitch, "pitch_range": pitch_range,
                "resolution": resolution,
            }, result)
            return result

        if not self.enable_real_ik:
            msg = "real execution requested but enable_real_ik is false"
            self._log(f"FAIL: {msg}")
            self._record("ik_solve", {
                "position": position, "rpy": rpy or [0, 0, 0],
            }, error=msg)
            return None

        if not _HAS_IK_SRV:
            msg = "kinematics_msgs not available"
            self._log(f"FAIL: {msg}")
            self._record("ik_solve", {
                "position": position, "rpy": rpy or [0, 0, 0],
            }, error=msg)
            return None

        try:
            self._log(f"  IK request → position={position} pitch={pitch} "
                      f"pitch_range={pitch_range} resolution={resolution}")

            # Offload blocking ROS2 service call to a background thread.
            # Creates a dedicated temporary rclpy node each call to avoid
            # executor conflicts with the main real_grounded_runtime_node.
            res = await asyncio.wait_for(
                asyncio.to_thread(
                    self._call_ik_blocking,
                    position, pitch, pitch_range, resolution, timeout_sec,
                ),
                timeout=timeout_sec + 1.0,
            )
        except asyncio.TimeoutError:
            msg = f"IK timeout after {timeout_sec}s"
            self._log(f"FAIL: {msg}  target={position}")
            self._record("ik_solve", {
                "position": position, "pitch": pitch,
                "pitch_range": pitch_range,
            }, error=msg)
            return None
        except Exception as e:
            msg = f"IK exception: {e}"
            self._log(f"FAIL: {msg}")
            self._record("ik_solve", {
                "position": position, "rpy": rpy or [0, 0, 0],
            }, error=msg)
            return None

        if res is None:
            msg = "IK service returned None"
            self._log(f"FAIL: {msg}")
            self._record("ik_solve", {
                "position": position, "pitch": pitch,
            }, error=msg)
            return None

        self._log(
            f"  IK response → success={res.success}"
            + (f" pulse={list(res.pulse)}" if hasattr(res, 'pulse') and res.pulse else "")
            + (f" rpy={list(res.rpy)}" if hasattr(res, 'rpy') and res.rpy else "")
        )

        if not res.success:
            msg = "IK service returned success=false"
            self._log(f"FAIL: {msg}")
            self._record("ik_solve", {
                "position": position, "pitch": pitch,
                "pitch_range": pitch_range,
            }, error=msg)
            return None

        if hasattr(res, 'pulse') and res.pulse:
            pulses = [int(p) for p in list(res.pulse)]
            self._record("ik_solve", {
                "position": position, "pitch": pitch,
                "pitch_range": pitch_range, "resolution": resolution,
            }, {"pulse": pulses,
                "rpy": list(res.rpy) if hasattr(res, 'rpy') and res.rpy else None})
            self._log(f"ik_solve OK → pulses={pulses}")
            return pulses

        msg = "IK response has no pulse field"
        self._log(f"FAIL: {msg}")
        self._record("ik_solve", {
            "position": position, "pitch": pitch,
        }, error=msg)
        return None

    def _call_ik_blocking(self, position, pitch, pitch_range,
                          resolution, timeout_sec):
        import rclpy
        self._log("  creating temp IK node ...")
        ik_node = rclpy.create_node("runtime_adapter_ik_client")
        try:
            client = ik_node.create_client(
                IKSetRobotPose, "/kinematics/set_pose_target"
            )
            if not client.wait_for_service(timeout_sec=3.0):
                self._log("  WARN: temp IK client wait_for_service failed")
                return None
            self._log("  temp IK client wait_for_service OK")

            req = IKSetRobotPose.Request()
            req.position = [float(p) for p in position[:3]]
            req.pitch = float(pitch)
            req.pitch_range = [float(v) for v in pitch_range[:2]]
            req.resolution = float(resolution)

            future = client.call_async(req)
            deadline = _time.time() + timeout_sec
            while rclpy.ok() and not future.done() and _time.time() < deadline:
                rclpy.spin_once(ik_node, timeout_sec=0.02)

            if not future.done():
                self._log(f"  IK future not done after {timeout_sec}s")
                return None

            return future.result()
        finally:
            ik_node.destroy_node()

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
