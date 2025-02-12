# Standard message header
std_msgs/Header header

# Possible button events
string PRESSED = "PRESSED"
string RELEASED = "RELEASED"
string NO_EVENT = ""

# Button names. The button event for the pressed button(s) should be set accordingly, or NO_EVENT if the button did not change.
string circle NO_EVENT
string cross NO_EVENT
string check NO_EVENT
string left NO_EVENT
string right NO_EVENT
string up NO_EVENT
string down NO_EVENT