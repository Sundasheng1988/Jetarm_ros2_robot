from typing import List, Optional


class RuntimeAdapter:
    def __init__(self, node=None, dry_run: bool = True):
        self._node = node
        self.dry_run = dry_run
        self._log_fn = None
        if node is not None:
            self._log_fn = node.get_logger().info
        self._call_log: List[dict] = []

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
            f"resolution={resolution} dry_run={self.dry_run}"
        )
        if self.dry_run:
            result = [500, 500, 500, 500, 500]
            self._record("ik_solve", {
                "position": position, "rpy": rpy or [0, 0, 0],
                "pitch": pitch, "resolution": resolution,
            }, result)
            return result
        raise NotImplementedError("Real IK not available in non-dry_run mode")

    async def servo_move(self, pulses: List[int], duration_ms: int = 2000):
        self._log(
            f"servo_move pulses={pulses} duration_ms={duration_ms} "
            f"dry_run={self.dry_run}"
        )
        self._record("servo_move", {
            "pulses": pulses, "duration_ms": duration_ms,
        })
        if self.dry_run:
            return
        raise NotImplementedError("Real servo not available in non-dry_run mode")

    async def gripper_set(self, servo_id: int, pulse: int, duration_ms: int = 300):
        self._log(
            f"gripper_set servo_id={servo_id} pulse={pulse} "
            f"duration_ms={duration_ms} dry_run={self.dry_run}"
        )
        self._record("gripper_set", {
            "servo_id": servo_id, "pulse": pulse, "duration_ms": duration_ms,
        })
        if self.dry_run:
            return
        raise NotImplementedError("Real servo not available in non-dry_run mode")

    async def get_joint_state(self):
        self._log(f"get_joint_state dry_run={self.dry_run}")
        if self.dry_run:
            return None
        raise NotImplementedError("Real state not available in non-dry_run mode")

    async def query_vision(self, class_name: str = "", timeout_sec: float = 2.0):
        self._log(
            f"query_vision class_name={class_name} "
            f"timeout_sec={timeout_sec} dry_run={self.dry_run}"
        )
        if self.dry_run:
            return None
        raise NotImplementedError("Real vision not available in non-dry_run mode")
