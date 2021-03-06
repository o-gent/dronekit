
import copy
import logging
import time
from time import monotonic

from past.builtins import basestring
from pymavlink import mavwp
from pymavlink.dialects.v10 import ardupilotmega

from dronekit.atributes import *
from dronekit.command import CommandSequence
from dronekit.gimbal import Gimbal
from dronekit.observers import HasObservers
from dronekit.parameters import Parameters


class APIException(Exception):
    """
    Base class for DroneKit related exceptions.

    :param String message: Message string describing the exception
    """


class Vehicle(HasObservers):
    """
    The main vehicle API.

    Vehicle state is exposed through 'attributes' (e.g. :py:attr:`heading`). All attributes can be
    read, and some are also settable
    (:py:attr:`mode`, :py:attr:`armed` and :py:attr:`home_location`).

    Attributes can also be asynchronously monitored for changes by registering listener callback
    functions.

    Vehicle "settings" (parameters) are read/set using the :py:attr:`parameters` attribute.
    Parameters can be iterated and are also individually observable.

    Vehicle movement is primarily controlled using the :py:attr:`armed` attribute and
    :py:func:`simple_takeoff` and :py:func:`simple_goto` in GUIDED mode.

    It is also possible to work with vehicle "missions" using the :py:attr:`commands` attribute,
    and run them in AUTO mode.

    STATUSTEXT log messages from the autopilot are handled through a separate logger.
    It is possible to configure the log level, the formatting, etc. by accessing the logger, e.g.:

    .. code-block:: python

        import logging
        autopilot_logger = logging.getLogger('autopilot')
        autopilot_logger.setLevel(logging.DEBUG)

    The guide contains more detailed information on the different ways you can use
    the ``Vehicle`` class:

    - :doc:`guide/vehicle_state_and_parameters`
    - :doc:`guide/copter/guided_mode`
    - :doc:`guide/auto_mode`


    .. note::

        This class currently exposes just the attributes that are most commonly used by all
        vehicle types. if you need to add additional attributes then subclass ``Vehicle``
        as demonstrated in :doc:`examples/create_attribute`.

        Please then :doc:`contribute <contributing/contributions_api>` your additions back
        to the project!
    """

    def __init__(self, handler):
        super(Vehicle, self).__init__()

        self._logger = logging.getLogger(__name__)  # Logger for DroneKit
        self._autopilot_logger = logging.getLogger('autopilot')  # Logger for the autopilot messages
        # MAVLink-to-logging-module log severity mappings
        self._mavlink_statustext_severity = {
            0: logging.CRITICAL,
            1: logging.CRITICAL,
            2: logging.CRITICAL,
            3: logging.ERROR,
            4: logging.WARNING,
            5: logging.INFO,
            6: logging.INFO,
            7: logging.DEBUG
        }

        self._handler = handler
        self._master = handler.master

        # Cache all updated attributes for wait_ready.
        # By default, we presume all "commands" are loaded.
        self._ready_attrs = {'commands'}

        # Default parameters when calling wait_ready() or wait_ready(True).
        self._default_ready_attrs = ['parameters', 'gps_0', 'armed', 'mode', 'attitude']

        @self.on_attribute('*')
        def listener(_, name, value):
            self._ready_attrs.add(name)

        # Attaches message listeners.
        self._message_listeners = dict()

        @handler.forward_message
        def listener(_, msg):
            self.notify_message_listeners(msg.get_type(), msg)

        self._location = Locations(self)
        self._vx = None
        self._vy = None
        self._vz = None


        self._wind_direction = None
        self._wind_speed = None
        self._wind_speed_z = None

        @self.on_message('WIND')
        def listener(self,name, m):
            """ WIND {direction : -180.0, speed : 0.0, speed_z : 0.0} """
            self._wind_direction = m.direction
            self._wind_speed = m.speed
            self._wind_speed_z = m.speed_z


        @self.on_message('STATUSTEXT')
        def statustext_listener(self, name, m):
            # Log the STATUSTEXT on the autopilot logger, with the correct severity
            self._autopilot_logger.log(
                msg=m.text.strip(),
                level=self._mavlink_statustext_severity[m.severity]
            )

        @self.on_message('GLOBAL_POSITION_INT')
        def listener(self, name, m):
            (self._vx, self._vy, self._vz) = (m.vx / 100.0, m.vy / 100.0, m.vz / 100.0)
            self.notify_attribute_listeners('velocity', self.velocity)

        self._pitch = None
        self._yaw = None
        self._roll = None
        self._pitchspeed = None
        self._yawspeed = None
        self._rollspeed = None

        @self.on_message('ATTITUDE')
        def listener(self, name, m):
            self._pitch = m.pitch
            self._yaw = m.yaw
            self._roll = m.roll
            self._pitchspeed = m.pitchspeed
            self._yawspeed = m.yawspeed
            self._rollspeed = m.rollspeed
            self.notify_attribute_listeners('attitude', self.attitude)

        self._heading = None
        self._airspeed = None
        self._groundspeed = None

        @self.on_message('VFR_HUD')
        def listener(self, name, m):
            self._heading = m.heading
            self.notify_attribute_listeners('heading', self.heading)
            self._airspeed = m.airspeed
            self.notify_attribute_listeners('airspeed', self.airspeed)
            self._groundspeed = m.groundspeed
            self.notify_attribute_listeners('groundspeed', self.groundspeed)

        self._rngfnd_distance = None
        self._rngfnd_voltage = None

        @self.on_message('RANGEFINDER')
        def listener(self, name, m):
            self._rngfnd_distance = m.distance
            self._rngfnd_voltage = m.voltage
            self.notify_attribute_listeners('rangefinder', self.rangefinder)

        self._mount_pitch = None
        self._mount_yaw = None
        self._mount_roll = None

        @self.on_message('MOUNT_STATUS')
        def listener(self, name, m):
            self._mount_pitch = m.pointing_a / 100.0
            self._mount_roll = m.pointing_b / 100.0
            self._mount_yaw = m.pointing_c / 100.0
            self.notify_attribute_listeners('mount', self.mount_status)

        self._capabilities = None
        self._raw_version = None
        self._autopilot_version_msg_count = 0

        @self.on_message('AUTOPILOT_VERSION')
        def listener(vehicle, name, m):
            self._capabilities = m.capabilities
            self._raw_version = m.flight_sw_version
            self._autopilot_version_msg_count += 1
            if self._capabilities != 0 or self._autopilot_version_msg_count > 5:
                # ArduPilot <3.4 fails to send capabilities correctly
                # straight after boot, and even older versions send
                # this back as always-0.
                vehicle.remove_message_listener('HEARTBEAT', self.send_capabilities_request)
            self.notify_attribute_listeners('autopilot_version', self._raw_version)

        # gimbal
        self._gimbal = Gimbal(self)

        # All keys are strings.
        self._channels = Channels(self, 8)

        @self.on_message(['RC_CHANNELS_RAW', 'RC_CHANNELS'])
        def listener(self, name, m):
            def set_rc(chnum, v):
                '''Private utility for handling rc channel messages'''
                # use port to allow ch nums greater than 8
                port = 0 if name == "RC_CHANNELS" else m.port
                self._channels._update_channel(str(port * 8 + chnum), v)

            for i in range(1, (18 if name == "RC_CHANNELS" else 8)+1):
                set_rc(i, getattr(m, "chan{}_raw".format(i)))

            self.notify_attribute_listeners('channels', self.channels)

        self._voltage = None
        self._current = None
        self._level = None

        @self.on_message('SYS_STATUS')
        def listener(self, name, m):
            self._voltage = m.voltage_battery
            self._current = m.current_battery
            self._level = m.battery_remaining
            self.notify_attribute_listeners('battery', self.battery)

        self._eph = None
        self._epv = None
        self._satellites_visible = None
        self._fix_type = None  # FIXME support multiple GPSs per vehicle - possibly by using componentId

        @self.on_message('GPS_RAW_INT')
        def listener(self, name, m):
            self._eph = m.eph
            self._epv = m.epv
            self._satellites_visible = m.satellites_visible
            self._fix_type = m.fix_type
            self.notify_attribute_listeners('gps_0', self.gps_0)

        self._current_waypoint = 0

        @self.on_message(['WAYPOINT_CURRENT', 'MISSION_CURRENT'])
        def listener(self, name, m):
            self._current_waypoint = m.seq

        self._ekf_poshorizabs = False
        self._ekf_constposmode = False
        self._ekf_predposhorizabs = False

        @self.on_message('EKF_STATUS_REPORT')
        def listener(self, name, m):
            # boolean: EKF's horizontal position (absolute) estimate is good
            self._ekf_poshorizabs = (m.flags & ardupilotmega.EKF_POS_HORIZ_ABS) > 0
            # boolean: EKF is in constant position mode and does not know it's absolute or relative position
            self._ekf_constposmode = (m.flags & ardupilotmega.EKF_CONST_POS_MODE) > 0
            # boolean: EKF's predicted horizontal position (absolute) estimate is good
            self._ekf_predposhorizabs = (m.flags & ardupilotmega.EKF_PRED_POS_HORIZ_ABS) > 0

            self.notify_attribute_listeners('ekf_ok', self.ekf_ok, cache=True)

        self._flightmode = 'AUTO'
        self._armed = False
        self._system_status = None
        self._autopilot_type = None  # PX4, ArduPilot, etc.
        self._vehicle_type = None  # quadcopter, plane, etc.

        @self.on_message('HEARTBEAT')
        def listener(self, name, m):
            # ignore groundstations
            if m.type == mavutil.mavlink.MAV_TYPE_GCS:
                return
            self._armed = (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            self.notify_attribute_listeners('armed', self.armed, cache=True)
            self._autopilot_type = m.autopilot
            self._vehicle_type = m.type
            if self._is_mode_available(m.custom_mode, m.base_mode) is False:
                raise APIException("mode (%s, %s) not available on mavlink definition" % (m.custom_mode, m.base_mode))
            if self._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                self._flightmode = mavutil.interpret_px4_mode(m.base_mode, m.custom_mode)
            else:
                self._flightmode = self._mode_mapping_bynumber[m.custom_mode]
            self.notify_attribute_listeners('mode', self.mode, cache=True)
            self._system_status = m.system_status
            self.notify_attribute_listeners('system_status', self.system_status, cache=True)

        # Waypoints.

        self._home_location = None
        self._wploader = mavwp.MAVWPLoader()
        self._wp_loaded = True
        self._wp_uploaded = None
        self._wpts_dirty = False
        self._commands = CommandSequence(self)

        @self.on_message(['WAYPOINT_COUNT', 'MISSION_COUNT'])
        def listener(self, name, msg):
            if not self._wp_loaded:
                self._wploader.clear()
                self._wploader.expected_count = msg.count
                self._master.waypoint_request_send(0)

        @self.on_message(['HOME_POSITION'])
        def listener(self, name, msg):
            self._home_location = LocationGlobal(msg.latitude / 1.0e7, msg.longitude / 1.0e7, msg.altitude / 1000.0)
            self.notify_attribute_listeners('home_location', self.home_location, cache=True)

        @self.on_message(['WAYPOINT', 'MISSION_ITEM'])
        def listener(self, name, msg):
            if not self._wp_loaded:
                if msg.seq == 0:
                    if not (msg.x == 0 and msg.y == 0 and msg.z == 0):
                        self._home_location = LocationGlobal(msg.x, msg.y, msg.z)

                if msg.seq > self._wploader.count():
                    # Unexpected waypoint
                    pass
                elif msg.seq < self._wploader.count():
                    # Waypoint duplicate
                    pass
                else:
                    self._wploader.add(msg)

                    if msg.seq + 1 < self._wploader.expected_count:
                        self._master.waypoint_request_send(msg.seq + 1)
                    else:
                        self._wp_loaded = True
                        self.notify_attribute_listeners('commands', self.commands)

        # Waypoint send to master
        @self.on_message(['WAYPOINT_REQUEST', 'MISSION_REQUEST'])
        def listener(self, name, msg):
            if self._wp_uploaded is not None:
                wp = self._wploader.wp(msg.seq)
                handler.fix_targets(wp)
                self._master.mav.send(wp)
                self._wp_uploaded[msg.seq] = True

        # TODO: Waypoint loop listeners

        # Parameters.

        start_duration = 0.2
        repeat_duration = 1

        self._params_count = -1
        self._params_set = []
        self._params_loaded = False
        self._params_start = False
        self._params_map = {}
        self._params_last = monotonic.monotonic()  # Last new param.
        self._params_duration = start_duration
        self._parameters = Parameters(self)

        @handler.forward_loop
        def listener(_):
            # Check the time duration for last "new" params exceeds watchdog.
            if not self._params_start:
                return

            if not self._params_loaded and all(x is not None for x in self._params_set):
                self._params_loaded = True
                self.notify_attribute_listeners('parameters', self.parameters)

            if not self._params_loaded and monotonic.monotonic() - self._params_last > self._params_duration:
                c = 0
                for i, v in enumerate(self._params_set):
                    if v is None:
                        self._master.mav.param_request_read_send(0, 0, b'', i)
                        c += 1
                        if c > 50:
                            break
                self._params_duration = repeat_duration
                self._params_last = monotonic.monotonic()

        @self.on_message(['PARAM_VALUE'])
        def listener(self, name, msg):
            # If we discover a new param count, assume we
            # are receiving a new param set.
            if self._params_count != msg.param_count:
                self._params_loaded = False
                self._params_start = True
                self._params_count = msg.param_count
                self._params_set = [None] * msg.param_count

            # Attempt to set the params. We throw an error
            # if the index is out of range of the count or
            # we lack a param_id.
            try:
                if msg.param_index < msg.param_count and msg:
                    if self._params_set[msg.param_index] is None:
                        self._params_last = monotonic.monotonic()
                        self._params_duration = start_duration
                    self._params_set[msg.param_index] = msg

                self._params_map[msg.param_id] = msg.param_value
                self._parameters.notify_attribute_listeners(msg.param_id, msg.param_value,
                                                            cache=True)
            except:
                import traceback
                traceback.print_exc()

        # Heartbeats.

        self._heartbeat_started = False
        self._heartbeat_lastsent = 0
        self._heartbeat_lastreceived = 0
        self._heartbeat_timeout = False

        self._heartbeat_warning = 5
        self._heartbeat_error = 30
        self._heartbeat_system = None

        @handler.forward_loop
        def listener(_):
            # Send 1 heartbeat per second
            if monotonic.monotonic() - self._heartbeat_lastsent > 1:
                self._master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                self._heartbeat_lastsent = monotonic.monotonic()

            # Timeouts.
            if self._heartbeat_started:
                if self._heartbeat_error and monotonic.monotonic() - self._heartbeat_lastreceived > self._heartbeat_error > 0:
                    raise APIException('No heartbeat in %s seconds, aborting.' %
                                       self._heartbeat_error)
                elif monotonic.monotonic() - self._heartbeat_lastreceived > self._heartbeat_warning:
                    if self._heartbeat_timeout is False:
                        self._logger.warning('Link timeout, no heartbeat in last %s seconds' % self._heartbeat_warning)
                        self._heartbeat_timeout = True

        @self.on_message(['HEARTBEAT'])
        def listener(self, name, msg):
            # ignore groundstations
            if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                return
            self._heartbeat_system = msg.get_srcSystem()
            self._heartbeat_lastreceived = monotonic.monotonic()
            if self._heartbeat_timeout:
                self._logger.info('...link restored.')
            self._heartbeat_timeout = False

        self._last_heartbeat = None

        @handler.forward_loop
        def listener(_):
            if self._heartbeat_lastreceived:
                self._last_heartbeat = monotonic.monotonic() - self._heartbeat_lastreceived
                self.notify_attribute_listeners('last_heartbeat', self.last_heartbeat)

    @property
    def last_heartbeat(self):
        """
        Time since last MAVLink heartbeat was received (in seconds).

        The attribute can be used to monitor link activity and implement script-specific timeout handling.

        For example, to pause the script if no heartbeat is received for more than 1 second you might implement
        the following observer, and use ``pause_script`` in a program loop to wait until the link is recovered:

        .. code-block:: python

            pause_script=False

            @vehicle.on_attribute('last_heartbeat')
            def listener(self, attr_name, value):
                global pause_script
                if value > 1 and not pause_script:
                    print "Pausing script due to bad link"
                    pause_script=True;
                if value < 1 and pause_script:
                    pause_script=False;
                    print "Un-pausing script"

        The observer will be called at the period of the messaging loop (about every 0.01 seconds). Testing
        on SITL indicates that ``last_heartbeat`` averages about .5 seconds, but will rarely exceed 1.5 seconds
        when connected. Whether heartbeat monitoring can be useful will very much depend on the application.


        .. note::

            If you just want to change the heartbeat timeout you can modify the ``heartbeat_timeout``
            parameter passed to the :py:func:`connect() <dronekit.connect>` function.

        """
        return self._last_heartbeat

    def on_message(self, name):
        """
        Decorator for message listener callback functions.

        .. tip::

            This is the most elegant way to define message listener callback functions.
            Use :py:func:`add_message_listener` only if you need to be able to
            :py:func:`remove the listener <remove_message_listener>` later.

        A decorated message listener function is called with three arguments every time the
        specified message is received:

        * ``self`` - the current vehicle.
        * ``name`` - the name of the message that was intercepted.
        * ``message`` - the actual message (a `pymavlink <http://www.qgroundcontrol.org/mavlink/pymavlink>`_
          `class <https://www.samba.org/tridge/UAV/pymavlink/apidocs/classIndex.html>`_).

        For example, in the fragment below ``my_method`` will be called for every heartbeat message:

        .. code:: python

            @vehicle.on_message('HEARTBEAT')
            def my_method(self, name, msg):
                pass

        See :ref:`mavlink_messages` for more information.

        :param String name: The name of the message to be intercepted by the decorated listener function (or '*' to get all messages).
        """

        def decorator(fn):
            if isinstance(name, list):
                for n in name:
                    self.add_message_listener(n, fn)
            else:
                self.add_message_listener(name, fn)

        return decorator

    def add_message_listener(self, name, fn):
        """
        Adds a message listener function that will be called every time the specified message is received.

        .. tip::

            We recommend you use :py:func:`on_message` instead of this method as it has a more elegant syntax.
            This method is only preferred if you need to be able to
            :py:func:`remove the listener <remove_message_listener>`.

        The callback function must have three arguments:

        * ``self`` - the current vehicle.
        * ``name`` - the name of the message that was intercepted.
        * ``message`` - the actual message (a `pymavlink <http://www.qgroundcontrol.org/mavlink/pymavlink>`_
          `class <https://www.samba.org/tridge/UAV/pymavlink/apidocs/classIndex.html>`_).

        For example, in the fragment below ``my_method`` will be called for every heartbeat message:

        .. code:: python

            #Callback method for new messages
            def my_method(self, name, msg):
                pass

            vehicle.add_message_listener('HEARTBEAT',my_method)

        See :ref:`mavlink_messages` for more information.

        :param String name: The name of the message to be intercepted by the listener function (or '*' to get all messages).
        :param fn: The listener function that will be called if a message is received.
        """
        name = str(name)
        if name not in self._message_listeners:
            self._message_listeners[name] = []
        if fn not in self._message_listeners[name]:
            self._message_listeners[name].append(fn)

    def remove_message_listener(self, name, fn):
        """
        Removes a message listener (that was previously added using :py:func:`add_message_listener`).

        See :ref:`mavlink_messages` for more information.

        :param String name: The name of the message for which the listener is to be removed (or '*' to remove an 'all messages' observer).
        :param fn: The listener callback function to remove.

        """
        name = str(name)
        if name in self._message_listeners:
            if fn in self._message_listeners[name]:
                self._message_listeners[name].remove(fn)
                if len(self._message_listeners[name]) == 0:
                    del self._message_listeners[name]

    def notify_message_listeners(self, name, msg):
        for fn in self._message_listeners.get(name, []):
            try:
                fn(self, name, msg)
            except Exception:
                self._logger.exception('Exception in message handler for %s' % msg.get_type(), exc_info=True)

        for fn in self._message_listeners.get('*', []):
            try:
                fn(self, name, msg)
            except Exception:
                self._logger.exception('Exception in message handler for %s' % msg.get_type(), exc_info=True)

    def close(self):
        return self._handler.close()

    def flush(self):
        """
        Call ``flush()`` after :py:func:`adding <CommandSequence.add>` or :py:func:`clearing <CommandSequence.clear>` mission commands.

        After the return from ``flush()`` any writes are guaranteed to have completed (or thrown an
        exception) and future reads will see their effects.

        .. warning::

            This method is deprecated. It has been replaced by
            :py:func:`Vehicle.commands.upload() <Vehicle.commands.upload>`.
        """
        return self.commands.upload()

    #
    # Private sugar methods
    #

    @property
    def _mode_mapping(self):
        return self._master.mode_mapping()

    @property
    def _mode_mapping_bynumber(self):
        return mavutil.mode_mapping_bynumber(self._vehicle_type)

    def _is_mode_available(self, custommode_code, basemode_code=0):
        try:
            if self._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                mode = mavutil.interpret_px4_mode(basemode_code, custommode_code)
                return mode in self._mode_mapping
            return custommode_code in self._mode_mapping_bynumber
        except:
            return False

    #
    # Operations to support the standard API.
    #

    @property
    def mode(self):
        """
        This attribute is used to get and set the current flight mode. You
        can specify the value as a :py:class:`VehicleMode`, like this:

        .. code-block:: python

           vehicle.mode = VehicleMode('LOITER')

        Or as a simple string:

        .. code-block:: python

            vehicle.mode = 'LOITER'

        If you are targeting ArduPilot you can also specify the flight mode
        using a numeric value (this will not work with PX4 autopilots):

        .. code-block:: python

            # set mode to LOITER
            vehicle.mode = 5
        """
        if not self._flightmode:
            return None
        return VehicleMode(self._flightmode)

    @mode.setter
    def mode(self, v):
        if isinstance(v, basestring):
            v = VehicleMode(v)

        if self._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
            self._master.set_mode(v.name)
        elif isinstance(v, int):
            self._master.set_mode(v)
        else:
            self._master.set_mode(self._mode_mapping[v.name])

    @property
    def location(self):
        """
        The vehicle location in global, global relative and local frames (:py:class:`Locations`).

        The different frames are accessed through its members:

        * :py:attr:`global_frame <dronekit.Locations.global_frame>` (:py:class:`LocationGlobal`)
        * :py:attr:`global_relative_frame <dronekit.Locations.global_relative_frame>` (:py:class:`LocationGlobalRelative`)
        * :py:attr:`local_frame <dronekit.Locations.local_frame>` (:py:class:`LocationLocal`)

        For example, to print the location in each frame for a ``vehicle``:

        .. code-block:: python

            # Print location information for `vehicle` in all frames (default printer)
            print "Global Location: %s" % vehicle.location.global_frame
            print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
            print "Local Location: %s" % vehicle.location.local_frame    #NED

            # Print altitudes in the different frames (see class definitions for other available information)
            print "Altitude (global frame): %s" % vehicle.location.global_frame.alt
            print "Altitude (global relative frame): %s" % vehicle.location.global_relative_frame.alt
            print "Altitude (NED frame): %s" % vehicle.location.local_frame.down

        .. note::

            All the location "values" (e.g. ``global_frame.lat``) are initially
            created with value ``None``. The ``global_frame``, ``global_relative_frame``
            latitude and longitude values are populated shortly after initialisation but
            ``global_frame.alt`` may take a few seconds longer to be updated.
            The ``local_frame`` does not populate until the vehicle is armed.

        The attribute and its members are observable. To watch for changes in all frames using a listener
        created using a decorator (you can also define a listener and explicitly add it).

        .. code-block:: python

            @vehicle.on_attribute('location')
            def listener(self, attr_name, value):
                # `self`: :py:class:`Vehicle` object that has been updated.
                # `attr_name`: name of the observed attribute - 'location'
                # `value` is the updated attribute value (a :py:class:`Locations`). This can be queried for the frame information
                print " Global: %s" % value.global_frame
                print " GlobalRelative: %s" % value.global_relative_frame
                print " Local: %s" % value.local_frame

        To watch for changes in just one attribute (in this case ``global_frame``):

        .. code-block:: python

            @vehicle.on_attribute('location.global_frame')
            def listener(self, attr_name, value):
                # `self`: :py:class:`Locations` object that has been updated.
                # `attr_name`: name of the observed attribute - 'global_frame'
                # `value` is the updated attribute value.
                print " Global: %s" % value

            #Or watch using decorator: ``@vehicle.location.on_attribute('global_frame')``.
        """
        return self._location

    @property
    def wind(self):
        """
        Current wind status (:pu:class: `Wind`)
        """
        return Wind(self._wind_direction, self._wind_speed, self._wind_speed_z)

    @property
    def battery(self):
        """
        Current system batter status (:py:class:`Battery`).
        """
        if self._voltage is None or self._current is None or self._level is None:
            return None
        return Battery(self._voltage, self._current, self._level)

    @property
    def rangefinder(self):
        """
        Rangefinder distance and voltage values (:py:class:`Rangefinder`).
        """
        return Rangefinder(self._rngfnd_distance, self._rngfnd_voltage)

    @property
    def velocity(self):
        """
        Current velocity as a three element list ``[ vx, vy, vz ]`` (in meter/sec).
        """
        return [self._vx, self._vy, self._vz]

    @property
    def version(self):
        """
        The autopilot version and type in a :py:class:`Version` object.

        .. versionadded:: 2.0.3
        """
        return Version(self._raw_version, self._autopilot_type, self._vehicle_type)

    @property
    def capabilities(self):
        """
        The autopilot capabilities in a :py:class:`Capabilities` object.

        .. versionadded:: 2.0.3
        """
        return Capabilities(self._capabilities)

    @property
    def attitude(self):
        """
        Current vehicle attitude - pitch, yaw, roll (:py:class:`Attitude`).
        """
        return Attitude(self._pitch, self._yaw, self._roll)

    @property
    def gps_0(self):
        """
        GPS position information (:py:class:`GPSInfo`).
        """
        return GPSInfo(self._eph, self._epv, self._fix_type, self._satellites_visible)

    @property
    def armed(self):
        """
        This attribute can be used to get and set the ``armed`` state of the vehicle (``boolean``).

        The code below shows how to read the state, and to arm/disarm the vehicle:

        .. code:: python

            # Print the armed state for the vehicle
            print "Armed: %s" % vehicle.armed

            # Disarm the vehicle
            vehicle.armed = False

            # Arm the vehicle
            vehicle.armed = True
        """
        return self._armed

    @armed.setter
    def armed(self, value):
        if bool(value) != self._armed:
            if value:
                self._master.arducopter_arm()
            else:
                self._master.arducopter_disarm()

    @property
    def is_armable(self):
        """
        Returns ``True`` if the vehicle is ready to arm, false otherwise (``Boolean``).

        This attribute wraps a number of pre-arm checks, ensuring that the vehicle has booted,
        has a good GPS fix, and that the EKF pre-arm is complete.
        """
        # check that mode is not INITIALSING
        # check that we have a GPS fix
        # check that EKF pre-arm is complete
        return self.mode != 'INITIALISING' and (self.gps_0.fix_type is not None and self.gps_0.fix_type > 1) and self._ekf_predposhorizabs

    @property
    def system_status(self):
        """
        System status (:py:class:`SystemStatus`).

        The status has a ``state`` property with one of the following values:

        * ``UNINIT``: Uninitialized system, state is unknown.
        * ``BOOT``: System is booting up.
        * ``CALIBRATING``: System is calibrating and not flight-ready.
        * ``STANDBY``: System is grounded and on standby. It can be launched any time.
        * ``ACTIVE``: System is active and might be already airborne. Motors are engaged.
        * ``CRITICAL``: System is in a non-normal flight mode. It can however still navigate.
        * ``EMERGENCY``: System is in a non-normal flight mode. It lost control over parts
          or over the whole airframe. It is in mayday and going down.
        * ``POWEROFF``: System just initialized its power-down sequence, will shut down now.
        """
        return {
            0: SystemStatus('UNINIT'),
            1: SystemStatus('BOOT'),
            2: SystemStatus('CALIBRATING'),
            3: SystemStatus('STANDBY'),
            4: SystemStatus('ACTIVE'),
            5: SystemStatus('CRITICAL'),
            6: SystemStatus('EMERGENCY'),
            7: SystemStatus('POWEROFF'),
        }.get(self._system_status, None)

    @property
    def heading(self):
        """
        Current heading in degrees - 0..360, where North = 0 (``int``).
        """
        return self._heading

    @property
    def groundspeed(self):
        """
        Current groundspeed in metres/second (``double``).

        This attribute is settable. The set value is the default target groundspeed
        when moving the vehicle using :py:func:`simple_goto` (or other position-based
        movement commands).
        """
        return self._groundspeed

    @groundspeed.setter
    def groundspeed(self, speed):
        speed_type = 1  # ground speed
        msg = self.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,  # command
            0,  # confirmation
            speed_type,  # param 1
            speed,  # speed in metres/second
            -1, 0, 0, 0, 0  # param 3 - 7
        )

        # send command to vehicle
        self.send_mavlink(msg)

    @property
    def airspeed(self):
        """
        Current airspeed in metres/second (``double``).

        This attribute is settable. The set value is the default target airspeed
        when moving the vehicle using :py:func:`simple_goto` (or other position-based
        movement commands).
        """
        return self._airspeed

    @airspeed.setter
    def airspeed(self, speed):
        speed_type = 0  # air speed
        msg = self.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,  # command
            0,  # confirmation
            speed_type,  # param 1
            speed,  # speed in metres/second
            -1, 0, 0, 0, 0  # param 3 - 7
        )

        # send command to vehicle
        self.send_mavlink(msg)

    @property
    def gimbal(self):
        """
        Gimbal object for controlling, viewing and observing gimbal status (:py:class:`Gimbal`).

        .. versionadded:: 2.0.1
        """
        return self._gimbal

    @property
    def mount_status(self):
        """
        .. warning:: This method is deprecated. It has been replaced by :py:attr:`gimbal`.

        Current status of the camera mount (gimbal) as a three element list: ``[ pitch, yaw, roll ]``.

        The values in the list are set to ``None`` if no mount is configured.
        """
        return [self._mount_pitch, self._mount_yaw, self._mount_roll]

    @property
    def ekf_ok(self):
        """
        ``True`` if the EKF status is considered acceptable, ``False`` otherwise (``boolean``).
        """
        # legacy check for dronekit-python for solo
        # use same check that ArduCopter::system.pde::position_ok() is using
        if self.armed:
            return self._ekf_poshorizabs and not self._ekf_constposmode
        else:
            return self._ekf_poshorizabs or self._ekf_predposhorizabs

    @property
    def channels(self):
        """
        The RC channel values from the RC Transmitter (:py:class:`Channels`).

        The attribute can also be used to set and read RC Override (channel override) values
        via :py:attr:`Vehicle.channels.override <dronekit.Channels.overrides>`.

        For more information and examples see :ref:`example_channel_overrides`.

        To read the channels from the RC transmitter:

        .. code:: python

            # Get all channel values from RC transmitter
            print "Channel values from RC Tx:", vehicle.channels

            # Access channels individually
            print "Read channels individually:"
            print " Ch1: %s" % vehicle.channels['1']
            print " Ch2: %s" % vehicle.channels['2']

        """
        return self._channels

    @property
    def home_location(self):
        """
        The current home location (:py:class:`LocationGlobal`).

        To get the attribute you must first download the :py:func:`Vehicle.commands`.
        The attribute has a value of ``None`` until :py:func:`Vehicle.commands` has been downloaded
        **and** the autopilot has set an initial home location (typically where the vehicle first gets GPS lock).

        .. code-block:: python

            #Connect to a vehicle object (for example, on com14)
            vehicle = connect('com14', wait_ready=True)

            # Download the vehicle waypoints (commands). Wait until download is complete.
            cmds = vehicle.commands
            cmds.download()
            cmds.wait_ready()

            # Get the home location
            home = vehicle.home_location

        The ``home_location`` is not observable.

        The attribute can be written (in the same way as any other attribute) after it has successfully
        been populated from the vehicle. The value sent to the vehicle is cached in the attribute
        (and can potentially get out of date if you don't re-download ``Vehicle.commands``):

        .. warning::

            Setting the value will fail silently if the specified location is more than 50km from the EKF origin.


        """
        return copy.copy(self._home_location)

    @home_location.setter
    def home_location(self, pos):
        """
        Sets the home location (``LocationGlobal``).

        The value cannot be set until it has successfully been read from the vehicle. After being
        set the value is cached in the home_location attribute and does not have to be re-read.

        .. note::

            Setting the value will fail silently if the specified location is more than 50km from the EKF origin.
        """

        if not isinstance(pos, LocationGlobal):
            raise ValueError('Expecting home_location to be set to a LocationGlobal.')

        # Set cached home location.
        self._home_location = copy.copy(pos)

        # Send MAVLink update.
        self.send_mavlink(self.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,  # command
            0,  # confirmation
            0,  # param 1: 1 to use current position, 0 to use the entered values.
            0, 0, 0,  # params 2-4
            pos.lat, pos.lon, pos.alt))

    @property
    def commands(self):
        """
        Gets the editable waypoints/current mission for this vehicle (:py:class:`CommandSequence`).

        This can be used to get, create, and modify a mission.

        :returns: A :py:class:`CommandSequence` containing the waypoints for this vehicle.
        """
        return self._commands

    @property
    def parameters(self):
        """
        The (editable) parameters for this vehicle (:py:class:`Parameters`).
        """
        return self._parameters

    def wait_for(self, condition, timeout=None, interval=0.1, errmsg=None):
        '''Wait for a condition to be True.

        Wait for condition, a callable, to return True.  If timeout is
        nonzero, raise a TimeoutError(errmsg) if the condition is not
        True after timeout seconds.  Check the condition everal
        interval seconds.
        '''

        t0 = time.time()
        while not condition():
            t1 = time.time()
            if timeout and (t1 - t0) >= timeout:
                raise TimeoutError(errmsg)

            time.sleep(interval)

    def wait_for_armable(self, timeout=None):
        '''Wait for the vehicle to become armable.

        If timeout is nonzero, raise a TimeoutError if the vehicle
        is not armable after timeout seconds.
        '''

        def check_armable():
            return self.is_armable

        self.wait_for(check_armable, timeout=timeout)

    def arm(self, wait=True, timeout=None):
        '''Arm the vehicle.

        If wait is True, wait for arm operation to complete before
        returning.  If timeout is nonzero, raise a TimeouTerror if the
        vehicle has not armed after timeout seconds.
        '''

        self.armed = True

        if wait:
            self.wait_for(lambda: self.armed, timeout=timeout,
                          errmsg='failed to arm vehicle')

    def disarm(self, wait=True, timeout=None):
        '''Disarm the vehicle.

        If wait is True, wait for disarm operation to complete before
        returning.  If timeout is nonzero, raise a TimeouTerror if the
        vehicle has not disarmed after timeout seconds.
        '''
        self.armed = False

        if wait:
            self.wait_for(lambda: not self.armed, timeout=timeout,
                          errmsg='failed to disarm vehicle')

    def wait_for_mode(self, mode, timeout=None):
        '''Set the flight mode.

        If wait is True, wait for the mode to change before returning.
        If timeout is nonzero, raise a TimeoutError if the flight mode
        hasn't changed after timeout seconds.
        '''

        if not isinstance(mode, VehicleMode):
            mode = VehicleMode(mode)

        self.mode = mode

        self.wait_for(lambda: self.mode.name == mode.name,
                      timeout=timeout,
                      errmsg='failed to set flight mode')

    def wait_for_alt(self, alt, epsilon=0.1, rel=True, timeout=None):
        '''Wait for the vehicle to reach the specified altitude.

        Wait for the vehicle to get within epsilon meters of the
        given altitude.  If rel is True (the default), use the
        global_relative_frame. If rel is False, use the global_frame.
        If timeout is nonzero, raise a TimeoutError if the specified
        altitude has not been reached after timeout seconds.
        '''

        def get_alt():
            if rel:
                alt = self.location.global_relative_frame.alt
            else:
                alt = self.location.global_frame.alt

            return alt

        def check_alt():
            cur = get_alt()
            delta = abs(alt - cur)

            return (
                (delta < epsilon) or
                (cur > alt > start) or
                (cur < alt < start)
            )

        start = get_alt()

        self.wait_for(
            check_alt,
            timeout=timeout,
            errmsg='failed to reach specified altitude')

    def wait_simple_takeoff(self, alt=None, epsilon=0.1, timeout=None):
        self.simple_takeoff(alt)

        if alt is not None:
            self.wait_for_alt(alt, epsilon=epsilon, timeout=timeout)

    def simple_takeoff(self, alt=None):
        """
        Take off and fly the vehicle to the specified altitude (in metres) and then wait for another command.

        .. note::

            This function should only be used on Copter vehicles.


        The vehicle must be in GUIDED mode and armed before this is called.

        There is no mechanism for notification when the correct altitude is reached,
        and if another command arrives before that point (e.g. :py:func:`simple_goto`) it will be run instead.

        .. warning::

           Apps should code to ensure that the vehicle will reach a safe altitude before
           other commands are executed. A good example is provided in the guide topic :doc:`guide/taking_off`.

        :param alt: Target height, in metres.
        """
        if alt is not None:
            altitude = float(alt)
            if math.isnan(altitude) or math.isinf(altitude):
                raise ValueError("Altitude was NaN or Infinity. Please provide a real number")
            self._master.mav.command_long_send(0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                               0, 0, 0, 0, 0, 0, 0, altitude)

    def simple_goto(self, location, airspeed=None, groundspeed=None):
        '''
        Go to a specified global location (:py:class:`LocationGlobal` or :py:class:`LocationGlobalRelative`).

        There is no mechanism for notification when the target location is reached, and if another command arrives
        before that point that will be executed immediately.

        You can optionally set the desired airspeed or groundspeed (this is identical to setting
        :py:attr:`airspeed` or :py:attr:`groundspeed`). The vehicle will determine what speed to
        use if the values are not set or if they are both set.

        The method will change the :py:class:`VehicleMode` to ``GUIDED`` if necessary.

        .. code:: python

            # Set mode to guided - this is optional as the simple_goto method will change the mode if needed.
            vehicle.mode = VehicleMode("GUIDED")

            # Set the LocationGlobal to head towards
            a_location = LocationGlobal(-34.364114, 149.166022, 30)
            vehicle.simple_goto(a_location)

        :param location: The target location (:py:class:`LocationGlobal` or :py:class:`LocationGlobalRelative`).
        :param airspeed: Target airspeed in m/s (optional).
        :param groundspeed: Target groundspeed in m/s (optional).
        '''
        if isinstance(location, LocationGlobalRelative):
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            alt = location.alt
        elif isinstance(location, LocationGlobal):
            # This should be the proper code:
            # frame = mavutil.mavlink.MAV_FRAME_GLOBAL
            # However, APM discards information about the relative frame
            # and treats any alt value as relative. So we compensate here.
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            if not self.home_location:
                self.commands.download()
                self.commands.wait_ready()
            alt = location.alt - self.home_location.alt
        else:
            raise ValueError('Expecting location to be LocationGlobal or LocationGlobalRelative.')

        self._master.mav.mission_item_send(0, 0, 0, frame,
                                           mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0,
                                           0, 0, 0, location.lat, location.lon,
                                           alt)

        if airspeed is not None:
            self.airspeed = airspeed
        if groundspeed is not None:
            self.groundspeed = groundspeed

    def send_mavlink(self, message):
        """
        This method is used to send raw MAVLink "custom messages" to the vehicle.

        The function can send arbitrary messages/commands to the connected vehicle at any time and in any vehicle mode.
        It is particularly useful for controlling vehicles outside of missions (for example, in GUIDED mode).

        The :py:func:`message_factory <dronekit.Vehicle.message_factory>` is used to create messages in the appropriate format.

        For more information see the guide topic: :ref:`guided_mode_how_to_send_commands`.

        :param message: A ``MAVLink_message`` instance, created using :py:func:`message_factory <dronekit.Vehicle.message_factory>`.
            There is need to specify the system id, component id or sequence number of messages as the API will set these appropriately.
        """
        self._master.mav.send(message)

    @property
    def message_factory(self):
        """
        Returns an object that can be used to create 'raw' MAVLink messages that are appropriate for this vehicle.
        The message can then be sent using :py:func:`send_mavlink(message) <dronekit.Vehicle.send_mavlink>`.

        .. note::

            Vehicles support a subset of the messages defined in the MAVLink standard. For more information
            about the supported sets see wiki topics:
            `Copter Commands in Guided Mode <http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/>`_
            and `Plane Commands in Guided Mode <http://dev.ardupilot.com/wiki/plane-commands-in-guided-mode/>`_.

        All message types are defined in the central MAVLink github repository.  For example, a Pixhawk understands
        the following messages (from `pixhawk.xml <https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/pixhawk.xml>`_):

        .. code:: xml

          <message id="153" name="IMAGE_TRIGGER_CONTROL">
               <field type="uint8_t" name="enable">0 to disable, 1 to enable</field>
          </message>

        The name of the factory method will always be the lower case version of the message name with *_encode* appended.
        Each field in the XML message definition must be listed as arguments to this factory method.  So for this example
        message, the call would be:

        .. code:: python

            msg = vehicle.message_factory.image_trigger_control_encode(True)
            vehicle.send_mavlink(msg)

        Some message types include "addressing information". If present, there is no need to specify the ``target_system``
        id (just set to zero) as DroneKit will automatically update messages with the correct ID for the connected
        vehicle before sending.
        The ``target_component`` should be set to 0 (broadcast) unless the message is to specific component.
        CRC fields and sequence numbers (if defined in the message type) are automatically set by DroneKit and can also
        be ignored/set to zero.

        For more information see the guide topic: :ref:`guided_mode_how_to_send_commands`.
        """
        return self._master.mav

    def initialize(self, rate=4, heartbeat_timeout=30):
        self._handler.start()

        # Start heartbeat polling.
        start = monotonic.monotonic()
        self._heartbeat_error = heartbeat_timeout or 0
        self._heartbeat_started = True
        self._heartbeat_lastreceived = start

        # Poll for first heartbeat.
        # If heartbeat times out, this will interrupt.
        while self._handler._alive:
            time.sleep(.1)
            if self._heartbeat_lastreceived != start:
                break
        if not self._handler._alive:
            raise APIException('Timeout in initializing connection.')

        # Register target_system now.
        self._handler.target_system = self._heartbeat_system

        # Wait until board has booted.
        while True:
            if self._flightmode not in [None, 'INITIALISING', 'MAV']:
                break
            time.sleep(0.1)

        # Initialize data stream.
        if rate is not None:
            self._master.mav.request_data_stream_send(0, 0, mavutil.mavlink.MAV_DATA_STREAM_ALL,
                                                      rate, 1)

        self.add_message_listener('HEARTBEAT', self.send_capabilities_request)

        # Ensure initial parameter download has started.
        while True:
            # This fn actually rate limits itself to every 2s.
            # Just retry with persistence to get our first param stream.
            self._master.param_fetch_all()
            time.sleep(0.1)
            if self._params_count > -1:
                break

    def send_capabilties_request(self, vehicle, name, m):
        '''An alias for send_capabilities_request.

        The word "capabilities" was misspelled in previous versions of this code. This is simply
        an alias to send_capabilities_request using the legacy name.
        '''
        return self.send_capabilities_request(vehicle, name, m)

    def send_capabilities_request(self, vehicle, name, m):
        '''Request an AUTOPILOT_VERSION packet'''
        capability_msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, 0, 1, 0, 0, 0, 0, 0, 0)
        vehicle.send_mavlink(capability_msg)

    def play_tune(self, tune):
        '''Play a tune on the vehicle'''
        msg = self.message_factory.play_tune_encode(0, 0, tune)
        self.send_mavlink(msg)

    def wait_ready(self, *types, **kwargs):
        """
        Waits for specified attributes to be populated from the vehicle (values are initially ``None``).

        This is typically called "behind the scenes" to ensure that :py:func:`connect` does not return until
        attributes have populated (via the ``wait_ready`` parameter). You can also use it after connecting to
        wait on a specific value(s).

        There are two ways to call the method:

        .. code-block:: python

            #Wait on default attributes to populate
            vehicle.wait_ready(True)

            #Wait on specified attributes (or array of attributes) to populate
            vehicle.wait_ready('mode','airspeed')

        Using the ``wait_ready(True)`` waits on :py:attr:`parameters`, :py:attr:`gps_0`,
        :py:attr:`armed`, :py:attr:`mode`, and :py:attr:`attitude`. In practice this usually
        means that all supported attributes will be populated.

        By default, the method will timeout after 30 seconds and raise an exception if the
        attributes were not populated.

        :param types: ``True`` to wait on the default set of attributes, or a
            comma-separated list of the specific attributes to wait on.
        :param int timeout: Timeout in seconds after which the method will raise an exception
            (the default) or return ``False``. The default timeout is 30 seconds.
        :param Boolean raise_exception: If ``True`` the method will raise an exception on timeout,
            otherwise the method will return ``False``. The default is ``True`` (raise exception).
        """
        timeout = kwargs.get('timeout', 30)
        raise_exception = kwargs.get('raise_exception', True)

        # Vehicle defaults for wait_ready(True) or wait_ready()
        if list(types) == [True] or list(types) == []:
            types = self._default_ready_attrs

        if not all(isinstance(item, basestring) for item in types):
            raise ValueError('wait_ready expects one or more string arguments.')

        # Wait for these attributes to have been set.
        await_attributes = set(types)
        start = monotonic.monotonic()
        still_waiting_last_message_sent = start
        still_waiting_callback = kwargs.get('still_waiting_callback')
        still_waiting_message_interval = kwargs.get('still_waiting_interval', 1)

        while not await_attributes.issubset(self._ready_attrs):
            time.sleep(0.1)
            now = monotonic.monotonic()
            if now - start > timeout:
                if raise_exception:
                    raise TimeoutError('wait_ready experienced a timeout after %s seconds.' %
                                       timeout)
                else:
                    return False
            if (still_waiting_callback and
                    now - still_waiting_last_message_sent > still_waiting_message_interval):
                still_waiting_last_message_sent = now
                if still_waiting_callback:
                    still_waiting_callback(await_attributes - self._ready_attrs)

        return True

    def reboot(self):
        """Requests an autopilot reboot by sending a ``MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`` command."""

        reboot_msg = self.message_factory.command_long_encode(
            0, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,  # command
            0,  # confirmation
            1,  # param 1, autopilot (reboot)
            0,  # param 2, onboard computer (do nothing)
            0,  # param 3, camera (do nothing)
            0,  # param 4, mount (do nothing)
            0, 0, 0)  # param 5 ~ 7 not used

        self.send_mavlink(reboot_msg)

    def send_calibrate_gyro(self):
        """Request gyroscope calibration."""

        calibration_command = self.message_factory.command_long_encode(
            self._handler.target_system, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command
            0,  # confirmation
            1,  # param 1, 1: gyro calibration, 3: gyro temperature calibration
            0,  # param 2, 1: magnetometer calibration
            0,  # param 3, 1: ground pressure calibration
            0,  # param 4, 1: radio RC calibration, 2: RC trim calibration
            0,  # param 5, 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
            0,  # param 6, 2: airspeed calibration
            0,  # param 7, 1: ESC calibration, 3: barometer temperature calibration
        )
        self.send_mavlink(calibration_command)

    def send_calibrate_magnetometer(self):
        """Request magnetometer calibration."""

        # ArduPilot requires the MAV_CMD_DO_START_MAG_CAL command, only present in the ardupilotmega.xml definition
        if self._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            calibration_command = self.message_factory.command_long_encode(
                self._handler.target_system, 0,  # target_system, target_component
                mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL,  # command
                0,  # confirmation
                0,  # param 1, uint8_t bitmask of magnetometers (0 means all).
                1,  # param 2, Automatically retry on failure (0=no retry, 1=retry).
                1,  # param 3, Save without user input (0=require input, 1=autosave).
                0,  # param 4, Delay (seconds).
                0,  # param 5, Autoreboot (0=user reboot, 1=autoreboot).
                0,  # param 6, Empty.
                0,  # param 7, Empty.
            )
        else:
            calibration_command = self.message_factory.command_long_encode(
                self._handler.target_system, 0,  # target_system, target_component
                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command
                0,  # confirmation
                0,  # param 1, 1: gyro calibration, 3: gyro temperature calibration
                1,  # param 2, 1: magnetometer calibration
                0,  # param 3, 1: ground pressure calibration
                0,  # param 4, 1: radio RC calibration, 2: RC trim calibration
                0,  # param 5, 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
                0,  # param 6, 2: airspeed calibration
                0,  # param 7, 1: ESC calibration, 3: barometer temperature calibration
            )

        self.send_mavlink(calibration_command)

    def send_calibrate_accelerometer(self, simple=False):
        """Request accelerometer calibration.

        :param simple: if True, perform simple accelerometer calibration
        """

        calibration_command = self.message_factory.command_long_encode(
            self._handler.target_system, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command
            0,  # confirmation
            0,  # param 1, 1: gyro calibration, 3: gyro temperature calibration
            0,  # param 2, 1: magnetometer calibration
            0,  # param 3, 1: ground pressure calibration
            0,  # param 4, 1: radio RC calibration, 2: RC trim calibration
            4 if simple else 1,  # param 5, 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
            0,  # param 6, 2: airspeed calibration
            0,  # param 7, 1: ESC calibration, 3: barometer temperature calibration
        )
        self.send_mavlink(calibration_command)

    def send_calibrate_vehicle_level(self):
        """Request vehicle level (accelerometer trim) calibration."""

        calibration_command = self.message_factory.command_long_encode(
            self._handler.target_system, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command
            0,  # confirmation
            0,  # param 1, 1: gyro calibration, 3: gyro temperature calibration
            0,  # param 2, 1: magnetometer calibration
            0,  # param 3, 1: ground pressure calibration
            0,  # param 4, 1: radio RC calibration, 2: RC trim calibration
            2,  # param 5, 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
            0,  # param 6, 2: airspeed calibration
            0,  # param 7, 1: ESC calibration, 3: barometer temperature calibration
        )
        self.send_mavlink(calibration_command)

    def send_calibrate_barometer(self):
        """Request barometer calibration."""

        calibration_command = self.message_factory.command_long_encode(
            self._handler.target_system, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command
            0,  # confirmation
            0,  # param 1, 1: gyro calibration, 3: gyro temperature calibration
            0,  # param 2, 1: magnetometer calibration
            1,  # param 3, 1: ground pressure calibration
            0,  # param 4, 1: radio RC calibration, 2: RC trim calibration
            0,  # param 5, 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
            0,  # param 6, 2: airspeed calibration
            0,  # param 7, 1: ESC calibration, 3: barometer temperature calibration
        )
        self.send_mavlink(calibration_command)

