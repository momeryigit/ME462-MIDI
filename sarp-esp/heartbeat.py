import time
from machine import WDT

class Heartbeat:
    """
    A class to manage the watchdog timer (WDT) and send periodic heartbeat signals.
    
    Attributes:
    wdt : WDT
        The watchdog timer instance.
    sent : bool
        Flag indicating if a heartbeat signal has been sent.
    _internal_timer : int
        Internal timer to track time since the last heartbeat.
    _timeout : int
        Timeout duration for heartbeat signals (in milliseconds).
    active : bool
        Flag indicating if the heartbeat is active.
    """
    
    def __init__(self, timeout=7000):
        """
        Initializes the Heartbeat class with a specified timeout.

        Parameters:
        timeout (int): The timeout duration in milliseconds. 
        Max allowed is 8300ms. Default is 7000ms.
        """
        if timeout > 8300:
            print("Timeout value exceeds maximum allowed value of 8300ms. Set to default value of 7000ms.")
            timeout = 7000
        self.wdt = WDT(timeout=timeout)  # enable WDT with specified timeout
        self.sent = False
        self._internal_timer = time.ticks_ms()
        self._timeout = timeout / 2  # set heartbeat interval to half of WDT timeout
        self.active = True

    def beat(self):
        """
        Sends a heartbeat signal if the time since the last signal exceeds the timeout.
        """
        if not self.active:
            return
        
        # If the time since the last beat is greater than the timeout, send a heartbeat
        if time.ticks_diff(time.ticks_ms(), self._internal_timer) > self._timeout and not self.sent:
            print('h')
            self.sent = True
            self._internal_timer = time.ticks_ms()

    def feed(self):
        """
        Feeds the watchdog timer if a heartbeat signal has been sent, resetting the WDT.
        """
        if not self.active:
            return
        
        if self.sent:
            self.wdt.feed()
            self._internal_timer = time.ticks_ms()
            self.sent = False
            print("I'm fed")

    def virtual_reset(self):
        """
        Resets the internal timer and heartbeat flag if the time since the last signal exceeds twice the timeout.
        """
        if not self.active:
            return
        
        if time.ticks_diff(time.ticks_ms(), self._internal_timer) > self._timeout * 2 and self.sent:
            print("I am resetting")
            time.sleep(0.5)
            self._internal_timer = time.ticks_ms()
            self.sent = False

    def stop(self):
        """
        Stops the heartbeat by setting the active flag to False.
        """
        self.active = False
        print("Heartbeat stopped")

    def start(self):
        """
        Starts the heartbeat by setting the active flag to True and resetting the internal timer and state.
        """
        self.active = True
        self._internal_timer = time.ticks_ms()
        self.sent = False
        print("Heartbeat started")

    def is_active(self):
        """
        Checks if the heartbeat is currently active.

        Returns:
        bool: True if the heartbeat is active, False otherwise.
        """
        return self.active
