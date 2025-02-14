# Standard message header
std_msgs/Header header

# Possible buttons for which events are captured
# Arrow keys. These buttons are not captured when the FCI is used!
string UP = "up"
string DOWN = "down"
string LEFT = "left"
string RIGHT = "right"
# Action buttons.
string CIRCLE = "circle"
string CROSS = "cross"
string CHECK = "check"

# Which of the buttons are pressed or released. If no event occurred for a button, it is not included in either array.
string[<=7] pressed
string[<=7] released
