"""Franka Pilot Buttons node.

This node will read the Pilot Buttons state from the Franka Desk and publish to ROS 2 topic.
"""

import asyncio
import base64
import contextlib
import hashlib
import json
import os
import pathlib
import ssl
import typing
from urllib import parse

import rclpy
import requests
import urllib3
from dotenv import load_dotenv
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from websockets.asyncio.client import connect

from franka_buttons_interfaces.msg import FrankaPilotButtonEvent

DEFAULT_CREDENTIALS_DIRECTORY = pathlib.Path("~/.ros/franka_buttons/credentials").expanduser()
"""Path to the directory where credentials and tokens used for Franka Desk authentication are stored."""


PilotButtonEvent = dict[str, bool]


class Desk:
    """Connects to the control box running the web-based Desk interface to manage the robot.

    This is required when connecting to the Desk headlessly, i.e. purely from Python.

    Newer versions of the system software use role-based access management to allow only one user to be in control of
    the Desk at a time. Control of a Desk can only be taken, if there is no active claim or the controlling user
    explicitly relinquishes control from the Desk dashboard.
    """

    @staticmethod
    def encode_password(username: str, password: str) -> str:
        """Encode the password into the form needed to log into the Desk interface."""
        bytes_str = ",".join([str(b) for b in hashlib.sha256((f"{password}#{username}@franka").encode()).digest()])
        return base64.encodebytes(bytes_str.encode("utf-8")).decode("utf-8")

    def __init__(self, hostname: str) -> None:
        """Initialize a headless Desk instance to connecto to the Franka Desk.

        :param hostname: Hostname of the Desk
        """
        self.hostname = hostname

        # Set up a Session to store the authorization headers
        self._session = requests.Session()
        self._session.verify = False  # Do not verify the SSL certificate
        urllib3.disable_warnings()  # Ignore the error about SSL certificate verification

    @property
    def is_logged_in(self) -> bool:
        """Check if this user is logged in."""
        return "authorization" in self._session.cookies

    def login(self, username: str, password: str, timeout: float | None = None) -> None:
        """Log into the Desk.

        :param username: Username of the Desk account.
        :param password: Plain password of the Desk account. It will be encoded before beint sent to the Desk.
        """
        response = self._request(
            "post",
            "/admin/api/login",
            json={"login": username, "password": self.encode_password(username, password)},
            timeout=timeout,
        )

        # Store the authorization cookie for subsequent requests
        self._session.cookies.set("authorization", response.text)

    def _request(
        self,
        method: typing.Literal["post", "get", "delete"],
        url: str,
        json: dict[str, str] | None = None,
        timeout: float | None = None,
    ) -> requests.Response:
        """Make a request to the Desk."""
        request_func = getattr(self._session, method)
        response: requests.Response = request_func(
            parse.urljoin(f"https://{self.hostname}", url),
            json=json,
            timeout=timeout,
        )
        if response.status_code == requests.codes.unauthorized:
            msg = f"Login credentials are incorrect. Response: {response.text}"
            raise ConnectionError(msg)
        if response.status_code != requests.codes.ok:
            raise ConnectionError(response.text)

        return response

    def create_websocket_connection(self, timeout: float | None = None) -> connect:
        """Create a websocket connection to the Franka Desk for pilot button events (navigation events)."""
        if not self.is_logged_in:
            msg = "Cannot connect to websocket: not logged in. Please log in first."
            raise ConnectionError(msg)

        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ssl_context.check_hostname = False
        ssl_context.verify_mode = ssl.CERT_NONE
        return connect(
            f"wss://{self.hostname}/desk/api/navigation/events",
            ssl=ssl_context,
            additional_headers={"authorization": self._session.cookies["authorization"]},
            open_timeout=timeout,
        )


class FrankaPilotButtonsNode(Node):
    """Connects to the Franka Desk and exposes the pilot buttons to ROS 2."""

    def __init__(self) -> None:
        """Initialize the pilot buttons node."""
        super().__init__("franka_pilot_buttons")

        # Declare all parameters
        hostname_param = self.declare_parameter(
            "hostname",
            "",
            ParameterDescriptor(
                description="[REQUIRED] Franka Desk hostname",
                read_only=True,  # Hostname cannot be changed after the node is initialized
            ),
        )
        # Validate that the parameter was set
        if hostname_param.get_parameter_value().string_value == "":
            self.get_logger().error("The 'hostname' parameter is required but was not provided!")
            msg = "Missing required parameter: 'hostname'"
            raise RuntimeError(msg)

        self.declare_parameter(
            "credentials_filepath",
            str(DEFAULT_CREDENTIALS_DIRECTORY / ".env"),
            ParameterDescriptor(
                description="Filepath to the credentials environment file.",
                read_only=True,  # Credentials path cannot be changed after the node is initialized
            ),
        )

        self.declare_parameter(
            "request_timeout",
            2.0,
            ParameterDescriptor(
                description="Timeout in seconds for requests to the Franka Desk.",
            ),
        )

        # Create publishers
        self.button_event_publisher = self.create_publisher(FrankaPilotButtonEvent, "franka_pilot_button_event", 10)

        # Set up the Franka Desk.
        self.desk = Desk(hostname_param.get_parameter_value().string_value)

    async def start_desk_loop(self) -> None:
        """Start the loop which obtains messages from the Desk websocket and publishes to ROS 2.

        This function will loop forever, so it is designed to run with `asyncio` alongside a `rclpy.spin*()` variant.
        """
        # Connect to the desk and log in
        credentials_filepath = self.get_parameter("credentials_filepath").get_parameter_value().string_value
        load_dotenv(credentials_filepath)

        self.get_logger().info(f"Logging in to Franka Desk using credentials in '{credentials_filepath}'.")
        self.desk.login(
            username=os.environ["FRANKA_DESK_USERNAME"],
            password=os.environ["FRANKA_DESK_PASSWORD"],
            timeout=self.get_parameter("request_timeout").get_parameter_value().double_value,
        )
        self.get_logger().info("Franka Desk login succesful.")

        # Obtain the websocket connection
        ws_connection = self.desk.create_websocket_connection(
            timeout=self.get_parameter("request_timeout").get_parameter_value().double_value,
        )
        self.get_logger().info("Websocket to Desk opened.")

        async with ws_connection as websocket:
            while rclpy.ok():
                # TODO: Possibly handle websockets.exceptions.ConnectionClosedOK here?
                # See https://github.com/jellehierck/franka_buttons_ros2/issues/1
                event: PilotButtonEvent = json.loads(await websocket.recv())
                self.handle_pilot_button(event)

    def handle_pilot_button(self, event: PilotButtonEvent) -> None:
        """Handle a pilot button event and publish to the corresponding topic.

        All the Pilot buttons except for the `Pilot Mode` button can be captured. Make sure Pilot Mode is set to Desk
        instead of End-Effector to receive direction key events. You can change the Pilot mode by pressing the `Pilot
        Mode` button or changing the mode in the Desk. Events will be triggered while buttons are pressed down or
        released.

        :param event: Dictionary containing the pressed state of each button. The keys are the button names (`circle`,
            `cross`, `check`, `left`, `right`, `down`, and `up`) and the value is a boolean indcating pressed (`True`)
            or released (`False`).
        """
        self.get_logger().info(f"Received {event=}")
        button_event_msg = FrankaPilotButtonEvent()

        for button, pressed in event.items():
            if button == "down":
                button_event_msg.down = FrankaPilotButtonEvent.PRESSED if pressed else FrankaPilotButtonEvent.RELEASED
            elif button == "up":
                button_event_msg.up = FrankaPilotButtonEvent.PRESSED if pressed else FrankaPilotButtonEvent.RELEASED
            elif button == "right":
                button_event_msg.right = FrankaPilotButtonEvent.PRESSED if pressed else FrankaPilotButtonEvent.RELEASED
            elif button == "left":
                button_event_msg.left = FrankaPilotButtonEvent.PRESSED if pressed else FrankaPilotButtonEvent.RELEASED
            elif button == "circle":
                button_event_msg.circle = FrankaPilotButtonEvent.PRESSED if pressed else FrankaPilotButtonEvent.RELEASED
            elif button == "cross":
                button_event_msg.cross = FrankaPilotButtonEvent.PRESSED if pressed else FrankaPilotButtonEvent.RELEASED
            elif button == "check":
                button_event_msg.check = FrankaPilotButtonEvent.PRESSED if pressed else FrankaPilotButtonEvent.RELEASED
            else:
                self.get_logger().warn(f"Unhandled button event {event=}")

        self.button_event_publisher.publish(button_event_msg)


async def spin_async(node: Node) -> None:
    """Spin a ROS 2 executor asynchronously."""
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)  # Ensure that the spin_once does not block by setting timeout to 0
        await asyncio.sleep(0)


async def start_node_async(node: FrankaPilotButtonsNode) -> None:
    """Start the node asynchronously."""
    node.get_logger().info("Starting async loops")

    # Tasks to be run in parallel to prevent the ROS 2 excecutor and websocket loop from blocking each other
    tasks = [asyncio.create_task(node.start_desk_loop()), asyncio.create_task(spin_async(node))]

    # Start the asyncio tasks until one of them completes (either finishes or raises an exception)
    done, pending = await asyncio.wait(
        tasks,
        return_when=asyncio.FIRST_EXCEPTION,
    )

    # Check for exceptions and prevent "Task exception was never retrieved"
    for task in done:
        if exc := task.exception():
            raise exc

    # Cancel remaining tasks if one fails, silently discarding the CancelledError
    for task in pending:
        task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await task


def main(args: list[str] | None = None) -> None:
    """Start the node."""
    rclpy.init(args=args)
    node = FrankaPilotButtonsNode()

    try:
        asyncio.run(start_node_async(node))

    except KeyboardInterrupt:
        pass

    finally:
        # Cleanup
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
