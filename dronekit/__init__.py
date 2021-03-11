# DroneAPI module
"""
This is the API Reference for the DroneKit-Python API.

The main API is the :py:class:`Vehicle` class.
The code snippet below shows how to use :py:func:`connect` to obtain an instance of a connected vehicle:

.. code:: python

    from dronekit import connect

    # Connect to the Vehicle using "connection string" (in this case an address on network)
    vehicle = connect('127.0.0.1:14550', wait_ready=True)

:py:class:`Vehicle` provides access to vehicle *state* through python attributes
(e.g. :py:attr:`Vehicle.mode`)
and to settings/parameters though the :py:attr:`Vehicle.parameters` attribute.
Asynchronous notification on vehicle attribute changes is available by registering listeners/observers.

Vehicle movement is primarily controlled using the :py:attr:`Vehicle.armed` attribute and
:py:func:`Vehicle.simple_takeoff` and :py:attr:`Vehicle.simple_goto` in GUIDED mode.

Velocity-based movement and control over other vehicle features can be achieved using custom MAVLink messages
(:py:func:`Vehicle.send_mavlink`, :py:func:`Vehicle.message_factory`).

It is also possible to work with vehicle "missions" using the :py:attr:`Vehicle.commands` attribute, and run them in AUTO mode.

All the logging is handled through the builtin Python `logging` module.

A number of other useful classes and methods are listed below.

----
"""

import logging

from dronekit.vehicle import Vehicle


class ErrprinterHandler(logging.Handler):
    """Logging handler to support the deprecated `errprinter` argument to connect()"""

    def __init__(self, errprinter):
        logging.Handler.__init__(self)
        self.errprinter = errprinter

    def emit(self, record):
        msg = self.format(record)
        self.errprinter(msg)

def default_still_waiting_callback(atts):
    logging.getLogger(__name__).debug("Still waiting for data from vehicle: %s" % ','.join(atts))


def connect(ip,
            _initialize=True,
            wait_ready=None,
            timeout=30,
            still_waiting_callback=default_still_waiting_callback,
            still_waiting_interval=1,
            status_printer=None,
            vehicle_class=None,
            rate=4,
            baud=115200,
            heartbeat_timeout=30,
            source_system=255,
            source_component=0,
            use_native=False):
    """
    Returns a :py:class:`Vehicle` object connected to the address specified by string parameter ``ip``.
    Connection string parameters (``ip``) for different targets are listed in the :ref:`getting started guide <get_started_connecting>`.

    The method is usually called with ``wait_ready=True`` to ensure that vehicle parameters and (most) attributes are
    available when ``connect()`` returns.

    .. code:: python

        from dronekit import connect

        # Connect to the Vehicle using "connection string" (in this case an address on network)
        vehicle = connect('127.0.0.1:14550', wait_ready=True)

    :param String ip: :ref:`Connection string <get_started_connecting>` for target address - e.g. 127.0.0.1:14550.

    :param Bool/Array wait_ready: If ``True`` wait until all default attributes have downloaded before
        the method returns (default is ``None``).
        The default attributes to wait on are: :py:attr:`parameters`, :py:attr:`gps_0`,
        :py:attr:`armed`, :py:attr:`mode`, and :py:attr:`attitude`.

        You can also specify a named set of parameters to wait on (e.g. ``wait_ready=['system_status','mode']``).

        For more information see :py:func:`Vehicle.wait_ready <Vehicle.wait_ready>`.

    :param status_printer: (deprecated) method of signature ``def status_printer(txt)`` that prints
        STATUS_TEXT messages from the Vehicle and other diagnostic information.
        By default the status information is handled by the ``autopilot`` logger.
    :param Vehicle vehicle_class: The class that will be instantiated by the ``connect()`` method.
        This can be any sub-class of ``Vehicle`` (and defaults to ``Vehicle``).
    :param int rate: Data stream refresh rate. The default is 4Hz (4 updates per second).
    :param int baud: The baud rate for the connection. The default is 115200.
    :param int heartbeat_timeout: Connection timeout value in seconds (default is 30s).
        If a heartbeat is not detected within this time an exception will be raised.
    :param int source_system: The MAVLink ID of the :py:class:`Vehicle` object returned by this method (by default 255).
    :param int source_component: The MAVLink Component ID fo the :py:class:`Vehicle` object returned by this method (by default 0).
    :param bool use_native: Use precompiled MAVLink parser.

        .. note::

            The returned :py:class:`Vehicle` object acts as a ground control station from the
            perspective of the connected "real" vehicle. It will process/receive messages from the real vehicle
            if they are addressed to this ``source_system`` id. Messages sent to the real vehicle are
            automatically updated to use the vehicle's ``target_system`` id.

            It is *good practice* to assign a unique id for every system on the MAVLink network.
            It is possible to configure the autopilot to only respond to guided-mode commands from a specified GCS ID.

            The ``status_printer`` argument is deprecated. To redirect the logging from the library and from the
            autopilot, configure the ``dronekit`` and ``autopilot`` loggers using the Python ``logging`` module.


    :returns: A connected vehicle of the type defined in ``vehicle_class`` (a superclass of :py:class:`Vehicle`).
    """

    from dronekit.mavlink import MAVConnection

    if not vehicle_class:
        vehicle_class = Vehicle

    handler = MAVConnection(ip, baud=baud, source_system=source_system, source_component=source_component, use_native=use_native)
    vehicle = vehicle_class(handler)

    if status_printer:
        vehicle._autopilot_logger.addHandler(ErrprinterHandler(status_printer))

    if _initialize:
        vehicle.initialize(rate=rate, heartbeat_timeout=heartbeat_timeout)

    if wait_ready:
        if wait_ready is True:
            vehicle.wait_ready(still_waiting_interval=still_waiting_interval,
                               still_waiting_callback=still_waiting_callback,
                               timeout=timeout)
        else:
            vehicle.wait_ready(*wait_ready)

    return vehicle
