# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
Unitree GO1 Robot Interface Model - ROS Basic Objects
-
This module contains object definitions that simplify the process of creating a
new ROS Node, Publisher, and Subscriber.
'''
# =============================================================================


# =============================================================================
# Imports
# =============================================================================

# used for creating ROS publishers and subscribers
import rospy # type: ignore

# used for type hinting
from typing import (
    Any,
    Callable,
    Optional,
    TYPE_CHECKING,
)


# =============================================================================
# ROS Node Definition
# =============================================================================
class ROS_Node(object):
    '''
    ROS Node
    -
    Contains the methods used to create a ROS Node capable of publishing and
    subscribing data, as well as storing data and logging/handling that data.

    Attributes
    -
    - _debug : `bool`
        - Flag for whether or not debug logging is enabled.
    - _name : `str`
        - Unique identifying name of the ROS Node.

    Methods
    -
    - __init__(name, debug=False) : `None`
        - Constructor Method.
        - Used to create a new ROS Node.
    - init(anonymous=False) : `None`
        - Instance Method.
        - Initializes the current ROS Node using `rospy`.
    - log_debug(msg) : `None`
        - Class Method.
        - Creates a debug log message.
    - log_err(msg) : `None`
        - Class Method.
        - Creates an error (stderr) log message.
    - log_fatal(msg) : `None`
        - Class Method.
        - Creates a fatal (stderr) log message.
    - log_info(msg) : `None`
        - Class Method.
        - Creates a info (stdout) log message.
    - log_warn(msg) : `None`
        - Class Method.
        - Creates a warning (stderr) log message.

    Properties
    -
    - name : `str`
        - Readonly.
        - Unique identifying name of the ROS Node.
    '''

    # ===========
    # Constructor
    def __init__(self, name: str, debug: bool = False) -> None:
        '''
        ROS Node Constructor
        -
        Used to create a new ROS Node.

        Parameters
        -
        - name : `str`
            - Unique identifying name of the ROS Node.
        - debug : `bool`
            - Flag for whether or not debug logging is enabled.

        Returns
        -
        None
        '''

        # set debug flag
        self._debug: bool = debug
        ''' Flag for whether or not debug logging is enabled. '''

        # set node name
        self._name: str = name
        ''' Unique identifying name of the ROS Node. '''

    # ===============
    # Property - Name
    @property
    def name(self) -> str:
        ''' Unique identifying name of the ROS Node. '''
        return self._name
    
    # ===============
    # Initialize Node
    def init(self, anonymous: bool = False) -> None:
        '''
        Initialize Node
        -
        Initializes the current ROS Node using `rospy`.

        Parameters
        -
        - anonymous : `bool`
            - Flag for whether or not to use anonymous naming for the ROS Node.

        Returns
        -
        None
        '''

        # enable debugging
        rospy.init_node(
            self.name,
            anonymous = anonymous,
            log_level = rospy.DEBUG if self._debug else rospy.INFO
        )

    # =================
    # Log Debug Message
    @classmethod
    def log_debug(cls, msg: str) -> None:
        '''
        Log Debug Message
        -
        Creates a debug log message.

        Parameters
        -
        - msg : `str`
            - Message to be logged as a "debug" message.

        Returns
        -
        None
        '''

        # create a new debug log message
        rospy.logdebug(msg)

    # =================
    # Log Error Message
    @classmethod
    def log_error(cls, msg: str) -> None:
        '''
        Log Error Message
        -
        Creates a error log message.

        Parameters
        -
        - msg : `str`
            - Message to be logged as a "error" message.

        Returns
        -
        None
        '''

        # create a new error log message
        rospy.logerr(msg)

    # =================
    # Log Fatal Message
    @classmethod
    def log_fatal(cls, msg: str) -> None:
        '''
        Log Fatal Message
        -
        Creates a fatal log message.

        Parameters
        -
        - msg : `str`
            - Message to be logged as a "fatal" message.

        Returns
        -
        None
        '''

        # create a new fatal log message
        rospy.logfatal(msg)

    # ================
    # Log Info Message
    @classmethod
    def log_info(cls, msg: str) -> None:
        '''
        Log Info Message
        -
        Creates an info log message.

        Parameters
        -
        - msg : `str`
            - Message to be logged as an "info" message.

        Returns
        -
        None
        '''

        # create a new info log message
        rospy.loginfo(msg)

    # ===================
    # Log Warning Message
    @classmethod
    def log_warn(cls, msg: str) -> None:
        '''
        Log Warning Message
        -
        Creates a warning log message.

        Parameters
        -
        - msg : `str`
            - Message to be logged as a "warning" message.

        Returns
        -
        None
        '''

        # create a new warning log message
        rospy.logwarn(msg)


# =============================================================================
# ROS Publisher Definition
# =============================================================================
class ROS_Publisher(rospy.Publisher):
    '''
    ROS Publisher
    -
    Contains the methods used to create and implement a ROS Publisher. This has
    no custom-defined functionality, just type-hinted and documented versions
    of the original `rospy.Publisher` functionality.

    Methods
    -
    - __init__(name, data_class, queue_size=10, latch=False) : `None`
        - Constructor Method.
        - Creates a new ROS Publisher.
    - publish(msg) : `None`
        - Instance Method.
        - Send a ROS message to the given topic.
    '''

    # ===========
    # Constructor
    def __init__(
            self,
            name: str,
            data_class: Any,
            queue_size: int = 10,
            latch: bool = False
    ) -> None:
        '''
        ROS Publisher Contructor
        -
        Creates a new ROS Publisher.

        Parameters
        -
        - name : `str`
            - Topic that the ROS Publisher is associated with.
        - data_class : `Any`
            - Message data type that will be sent to the given topic.
        - queue_size : `int`
            - Indicates the number of messages that can be asynchronously
                queued. Defaults to `10`. If `0`, then there is not limit,
                which can cause issues.
        - latch : `bool`
            - Defaults to `False`. If `True`, the last message published will
                be "latched", meaning that any future subscribers will be sent
                the message immediately upon connection.

        Returns
        -
        None
        '''

        # call super constructor
        super().__init__(
            name = name,
            data_class = data_class,
            queue_size = queue_size,
            latch = latch
        )

    # ===============
    # Publish Message
    def publish(self, msg: Any) -> None:
        '''
        Publish ROS Message
        -
        Sends a ROS message to the given topic.

        Parameters
        -
        - msg : `Any`
            - Message data to be sent.

        Returns
        -
        None
        '''

        # call super method
        super().publish(msg)


# =============================================================================
# ROS Subscriber Definition
# =============================================================================
class ROS_Subscriber(rospy.Subscriber):
    '''
    ROS Subscriber
    -
    Contains the methods used to create and implement a ROS Subscriber. This
    has no custom-defined functionality, just type-hinted and documented
    versions of the original `rospy.Subscriber` functionality.

    Methods
    -
    - __init__(name, data_class, callback=None, callback_args=None) : `None`
        - Constructor Method.
        - Creates a new ROS Subscriber.
    '''

    # ===========
    # Constructor
    def __init__(
            self,
            name: str,
            data_class: Any,
            callback: Any = None,
            callback_args: Any = None
    ) -> None:
        '''
        ROS Subscriber Constructor
        -
        Creates a new ROS Subscriber.

        Parameters
        -
        - name : `str`
            - Topic that the ROS Subscriber is associated with.
        - data_class : `Any`
            - Message data type that will be sent to the given topic.
        - callback : `Any`
            - Callback function that will be called when a message is received.
            - Type: `fn(msg, Any)`
            - Example:
                ``` python
                sub1 = ROS_Subscriber(
                    ...,
                    callback = self._callback_func, 
                    callback_args = ('arg1', 2, 'arg3')
                )
                sub2 = ROS_Subscriber(
                    ...,
                    callback = _callback_func,
                    callback_args = (1, [], None, 'hello')
                )
                ```
        - callback_args : `Any`
            - A collection of arguments that will be passed to the callback
                function. 
        '''

        # call super constructor
        super().__init__(
            name = name,
            data_class = data_class,
            callback = callback,
            callback_args = callback_args
        )


# =============================================================================
# End of File
# =============================================================================
