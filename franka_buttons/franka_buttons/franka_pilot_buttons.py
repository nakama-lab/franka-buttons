"""Code originally inspiered by https://github.com/JeanElsner/panda-py/blob/main/src/panda_py/__init__.py and
https://github.com/franzesegiovanni/franka_buttons/blob/main/scripts/buttons_listener.py.

All the credits to the authors.

TODO: Add more description and proper credits
"""

import asyncio
import base64
import dataclasses
import hashlib
import json
import os
import pathlib
import ssl
import typing
from urllib import parse

import rclpy
import requests
from dotenv import load_dotenv
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from requests.packages import urllib3
from std_msgs.msg import Bool, Float32
from websocket import WebSocket, create_connection

# TODO: Make this a parameter? Or only the .env file in ButtonDesk?
CREDENTIALS_DIRECTORY = pathlib.Path("~/.ros2/franka_buttons/credentials").expanduser()
"""Path to the directory where credentials and tokens used for Franka Desk authentication are stored."""

TOKEN_PATH = CREDENTIALS_DIRECTORY / "token.conf"
"""Path to the configuration file holding known control tokens.

If :py:class:`Desk` is used to connect to a control unit's web interface and takes control, the generated token is
stored in this file under the unit's IP address or hostname.
"""


PilotButtonEvent = dict[str, bool]


class Desk:
    """Connects to the control box running the web-based Desk interface to manage the robot.

    This is required when connecting to the Desk headlessly, i.e. purely from Python.

    Newer versions of the system software use role-based access management to allow only one user to be in control of
    the Desk at a time. Control of a Desk can only be taken, if there is no active claim or the controlling user
    explicitly relinquishes control from the Desk dashboard.
    """

    @staticmethod
    def encode_password(username: str, password: str) -> bytes:
        """Encode the password into the form needed to log into the Desk interface."""
        bytes_str = ",".join([str(b) for b in hashlib.sha256((f"{password}#{username}@franka").encode()).digest()])
        return base64.encodebytes(bytes_str.encode("utf-8")).decode("utf-8")

    def __init__(self, hostname: str):
        """Initialize a headless Desk instance to connecto to the Franka Desk.

        :param hostname: Hostname of the Desk
        """
        self.hostname = hostname

        # Set up connection to the Franka Desk and attempt to log in.
        # urllib3.disable_warnings()
        self._session = requests.Session()
        self._session.verify = False  # Do not verify the SSL certificate

    @property
    def is_logged_in(self) -> bool:
        """Check if this user is logged in."""
        return self._session.cookies.get("authorization", default=None) is not None

    def login(self, username: str, password: str) -> None:
        """Log into the Desk.

        :param username: Username of the Desk account.
        :param password: Plain password of the Desk account. It will be encoded before beint sent to the Desk.
        """
        # TODO: Add better error handling for incorrect username / password, e.g. with a custom exception
        # TODO: Add error to docstring
        response = self._request(
            "post",
            "/admin/api/login",
            json={"login": username, "password": self.encode_password(username, password)},
        )

        # Store the authorization cookie for subsequent requests
        self._session.cookies.set("authorization", response.text)

    def _request(
        self,
        method: typing.Literal["post", "get", "delete"],
        url: str,
        json: dict[str, str] | None = None,
        headers: dict[str, str] | None = None,
        files: dict[str, str] | None = None,
    ) -> requests.Response:
        """Make a request to the Desk."""
        request_func = getattr(self._session, method)
        response: requests.Response = request_func(
            parse.urljoin(f"https://{self.hostname}", url),
            json=json,
            headers=headers,
            files=files,
        )
        if response.status_code != 200:
            raise ConnectionError(response.text)
        return response

    def create_websocket_connection(self) -> WebSocket:
        """Create a websocket connection to the Franka Desk for pilot button events (navigation events)."""
        if not self.is_logged_in():
            raise Exception("Not logged in")  # TODO: Replace with better error

        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
        return create_connection(
            f"wss://{self.hostname}/desk/api/navigation/events",
            ssl_context=ctx,
            additional_headers={"authorization": self._session.cookies.get("authorization")},
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
                description="Franka Desk hostname",
                read_only=True,  # Hostname cannot be changed after the node is initialized
            ),
        )

        # TODO: Add parameter for where to find the login credentials
        load_dotenv(CREDENTIALS_DIRECTORY / ".env")  # TODO: Make this a parameter?

        # Set up connection to the Franka Desk and attempt to log in.
        self.desk = Desk(hostname_param.get_parameter_value().string_value)
        username = os.getenv("FRANKA_DESK_USERNAME")
        password = os.getenv("FRANKA_DESK_PASSWORD")
        self.desk.login(username, password)
        self.get_logger().info("Franka Desk login succesful.")

        # TODO: Replace with custom ROS message
        self.button_x_publisher = self.create_publisher("franka_buttons/x", Float32, queue_size=10)
        # self.button_y_publisher = rospy.Publisher("franka_buttons/y", Float32, queue_size=10)
        # self.button_circle_publisher = rospy.Publisher("franka_buttons/circle", Bool, queue_size=10)
        # self.button_cross_publisher = rospy.Publisher("franka_buttons/cross", Bool, queue_size=10)
        # self.button_check_publisher = rospy.Publisher("franka_buttons/check", Bool, queue_size=10)

    async def start_spin(self) -> None:
        """Start spinning the node.

        This function handles ROS 2 callbacks and websocket events asynchronously, and should therefore be called with
        the proper `asyncio` action.
        """
        timeout = 1.0  # TODO: Make configurable

        # Obtain the websocket connection
        ws_connection = self.desk.create_websocket_connection()

        # Create an asynchronous receive function to obtain the next message from the websocket
        async def websocket_recv(timeout: float | None = None) -> str | bytes:
            loop = asyncio.get_event_loop()
            return await loop.run_in_executor(None, ws_connection.recv, timeout)

        while rclpy.ok():
            # Check if there are any ROS callbacks to perform without blocking
            rclpy.spin_once(self, timeout_sec=0)

            # Handle websocket messages asynchronously
            try:
                event = await asyncio.wait_for(websocket_recv(), timeout=timeout)
                if event:
                    self.handle_pilot_button(json.loads(event))
            except asyncio.TimeoutError:
                pass

    def handle_pilot_button(self, event: PilotButtonEvent) -> None:
        """Handle a pilot button event.

        All the Pilot buttons except for the `Pilot Mode` button can be captured. Make sure Pilot Mode is set to Desk
        instead of End-Effector to receive direction key events. You can change the Pilot mode by pressing the `Pilot
        Mode` button or changing the mode in the Desk. Events will be triggered while buttons are pressed down or
        released.

        :param event: contains the triggered buttons as keys. The values of those keys will depend on the kind of event,
            either True for a button pressed down or False when released. The possible buttons are: `circle`, `cross`,
            `check`, `left`, `right`, `down`, and `up`.
        :param event: _description_
        """
        self.get_logger().info(f"Received {event=}")
        feedback_x = 0
        feedback_y = 0
        feedback_circle = False
        feedback_cross = False
        feedback_check = False

        for button, pressed in event.items():
            if button == "down" and pressed is True:
                feedback_x = 1
            elif button == "up" and pressed is True:
                feedback_x = -1
            elif button == "right" and pressed is True:
                feedback_y = 1
            elif button == "left" and pressed is True:
                feedback_y = -1
            elif button == "circle" and pressed is True:
                feedback_circle = True
            elif button == "cross" and pressed is True:
                feedback_cross = True
            elif button == "check" and pressed is True:
                feedback_check = True

        msg_x = Float32(data=feedback_x)
        msg_y = Float32(data=feedback_y)
        msg_circle = Bool(data=feedback_circle)
        msg_cross = Bool(data=feedback_cross)
        msg_check = Bool(data=feedback_check)
        self.button_x_publisher.publish(msg_x)
        self.button_y_publisher.publish(msg_y)
        self.button_circle_publisher.publish(msg_circle)
        self.button_cross_publisher.publish(msg_cross)
        self.button_check_publisher.publish(msg_check)


def main(args: list[str] | None = None) -> None:
    """Start the node."""
    rclpy.init(args=args)
    node = FrankaPilotButtonsNode()

    # Start asyncio loop and ROS 2 loop in parallel
    loop = asyncio.get_event_loop()
    node_task = loop.create_task(node.start_spin())

    # Create a strong reference to the asynchronous task and discard it once it is finished
    async_tasks = set()
    async_tasks.add(node_task)
    node_task.add_done_callback(async_tasks.discard)

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()
        loop.close()


if __name__ == "__main__":
    main()
