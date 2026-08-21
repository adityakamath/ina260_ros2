"""Turns continuous /battery_state readings into discrete SetBool service events.

lekiwi_audio's indicator_node only ever passively watches existing services' own
_service_event introspection (see STATE_SERVICES there, and /emergency_stop, /twist_switch,
/waypoint_follow) - it never subscribes to sensor topics directly. This node is what makes
"battery low"/"battery full"/etc. a service-backed piece of state like everything else it
narrates: it hosts a SetBool per event, and calls that same service as its own client
whenever a genuine edge fires, purely to produce the request/response traffic indicator_node
already knows how to observe. The server side has no real side effect beyond existing for
introspection to see - callers here are trusted to only invoke set() on a genuine edge, not
every tick, since nothing de-duplicates repeated identical calls downstream.

Five events, four services (charging/discharging are the two sides of one toggle):
    /battery_charging  - true once on DISCHARGING -> CHARGING, false once on CHARGING -> DISCHARGING
    /battery_low       - true once percentage falls to/below low_battery_threshold
    /battery_critical  - true once percentage falls to/below critical_battery_threshold
    /battery_full      - true once percentage rises to/above full_battery_threshold

The charging/discharging edge only fires on a *direct* status transition - passing through
NOT_CHARGING or FULL in between suppresses both sides until the raw status returns to
CHARGING or DISCHARGING. low/critical/full are independent hysteresis watchers (see
_ThresholdWatcher) so a percentage oscillating near a threshold doesn't spam repeat calls;
each also has a "false" side for re-arming, but phrases.yaml only wires a phrase to "true" -
this narrates the requested once-per-edge behavior while keeping the underlying services
genuinely two-sided for any other future consumer.
"""

from typing import Callable

import rclpy
from rclpy._rclpy_pybind11.service_introspection import ServiceIntrospectionState
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, LivelinessPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_srvs.srv import SetBool

# Matches the QoS /emergency_stop, /twist_switch and /waypoint_follow configure on their own
# introspection - required for indicator_node's late-join replay to actually receive anything.
STATE_SERVICE_QOS = QoSProfile(
    depth=2,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    liveliness=LivelinessPolicy.AUTOMATIC,
    liveliness_lease_duration=Duration(seconds=1),
)


class _StateService:
    """Hosts a SetBool service and calls it as its own client to emit a state change.

    The server handler always succeeds - there's no real work to do beyond existing so
    that /<name>/_service_event carries the request. set() is fire-and-forget (no await,
    no blocking wait on the response): it's called from a synchronous topic callback, and
    since nothing here needs the result, waiting would only add reentrancy risk for no
    benefit - unlike bool_toggle_node's client, which awaits a *different* node's real
    response and therefore does need a reentrant callback group.
    """

    def __init__(self, node: Node, service_name: str):
        self._node = node
        self._service_name = service_name
        server = node.create_service(SetBool, service_name, self._handle_request)
        server.configure_introspection(
            node.get_clock(), STATE_SERVICE_QOS, ServiceIntrospectionState.CONTENTS
        )
        self._client = node.create_client(SetBool, service_name)

    def _handle_request(self, request, response):
        response.success = True
        response.message = ''
        return response

    def set(self, value: bool) -> None:
        if not self._client.service_is_ready():
            self._node.get_logger().warning(
                f'{self._service_name}: client not ready yet, skipping this update'
            )
            return
        self._client.call_async(SetBool.Request(data=value))


class _ThresholdWatcher:
    """Fires once per genuine hysteresis-debounced crossing of a percentage threshold.

    direction='falling': triggers when percentage drops to/below threshold; re-arms (and
    fires the reset side) once it climbs back above threshold + hysteresis.
    direction='rising': triggers when percentage climbs to/above threshold; re-arms once it
    drops back below threshold - hysteresis.
    """

    def __init__(
        self,
        threshold: float,
        hysteresis: float,
        direction: str,
        on_trigger: Callable[[], None],
        on_reset: Callable[[], None],
    ):
        assert direction in ('falling', 'rising')
        self._threshold = threshold
        self._hysteresis = hysteresis
        self._direction = direction
        self._on_trigger = on_trigger
        self._on_reset = on_reset
        self._armed = True

    def update(self, percentage: float) -> None:
        if self._direction == 'falling':
            triggers = self._armed and percentage <= self._threshold
            resets = not self._armed and percentage >= self._threshold + self._hysteresis
        else:
            triggers = self._armed and percentage >= self._threshold
            resets = not self._armed and percentage <= self._threshold - self._hysteresis

        if triggers:
            self._armed = False
            self._on_trigger()
        elif resets:
            self._armed = True
            self._on_reset()


class BatteryEventsNode(Node):

    def __init__(self, **kwargs):
        super().__init__('battery_events_node', **kwargs)

        self.declare_parameter('battery_state_topic', '/battery_state')
        self.declare_parameter('low_battery_threshold', 0.2)
        self.declare_parameter('low_battery_hysteresis', 0.05)
        self.declare_parameter('critical_battery_threshold', 0.1)
        self.declare_parameter('critical_battery_hysteresis', 0.03)
        self.declare_parameter('full_battery_threshold', 0.98)
        self.declare_parameter('full_battery_hysteresis', 0.02)

        low_threshold = self.get_parameter('low_battery_threshold').value
        critical_threshold = self.get_parameter('critical_battery_threshold').value
        full_threshold = self.get_parameter('full_battery_threshold').value
        if not critical_threshold < low_threshold < full_threshold:
            raise ValueError(
                f'thresholds must satisfy critical < low < full, got '
                f'critical={critical_threshold} low={low_threshold} full={full_threshold}'
            )

        self._battery_charging = _StateService(self, '/battery_charging')
        self._battery_low = _StateService(self, '/battery_low')
        self._battery_critical = _StateService(self, '/battery_critical')
        self._battery_full = _StateService(self, '/battery_full')

        self._low_watcher = _ThresholdWatcher(
            low_threshold,
            self.get_parameter('low_battery_hysteresis').value,
            'falling',
            on_trigger=lambda: self._battery_low.set(True),
            on_reset=lambda: self._battery_low.set(False),
        )
        self._critical_watcher = _ThresholdWatcher(
            critical_threshold,
            self.get_parameter('critical_battery_hysteresis').value,
            'falling',
            on_trigger=lambda: self._battery_critical.set(True),
            on_reset=lambda: self._battery_critical.set(False),
        )
        self._full_watcher = _ThresholdWatcher(
            full_threshold,
            self.get_parameter('full_battery_hysteresis').value,
            'rising',
            on_trigger=lambda: self._battery_full.set(True),
            on_reset=lambda: self._battery_full.set(False),
        )

        self._last_status = None

        battery_state_topic = self.get_parameter('battery_state_topic').value
        self.create_subscription(
            BatteryState, battery_state_topic, self._on_battery_state, 10
        )

        self.get_logger().info(
            f'battery_events_node ready: watching {battery_state_topic}, '
            f'low={low_threshold} critical={critical_threshold} full={full_threshold}'
        )

    def _on_battery_state(self, msg: BatteryState) -> None:
        status = msg.power_supply_status
        if (
            self._last_status == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            and status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
        ):
            self._battery_charging.set(True)
        elif (
            self._last_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
            and status == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        ):
            self._battery_charging.set(False)
        self._last_status = status

        percentage = msg.percentage
        if percentage == percentage:  # NaN = unmeasured, per the message's own contract
            self._low_watcher.update(percentage)
            self._critical_watcher.update(percentage)
            self._full_watcher.update(percentage)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryEventsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
