import math

from pymavlink import mavutil

from dronekit.observers import HasObservers


class Attitude(object):
    """
    Attitude information.

    An object of this type is returned by :py:attr:`Vehicle.attitude`.

    .. _figure_attitude:

    .. figure:: http://upload.wikimedia.org/wikipedia/commons/thumb/c/c1/Yaw_Axis_Corrected.svg/500px-Yaw_Axis_Corrected.svg.png
        :width: 400px
        :alt: Diagram showing Pitch, Roll, Yaw
        :target: http://commons.wikimedia.org/wiki/File:Yaw_Axis_Corrected.svg

        Diagram showing Pitch, Roll, Yaw (`Creative Commons <http://commons.wikimedia.org/wiki/File:Yaw_Axis_Corrected.svg>`_)

    :param pitch: Pitch in radians
    :param yaw: Yaw in radians
    :param roll: Roll in radians
    """

    def __init__(self, pitch, yaw, roll):
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll

    def __str__(self):
        fmt = '{}:pitch={pitch},yaw={yaw},roll={roll}'
        return fmt.format(self.__class__.__name__, **vars(self))


class LocationGlobal(object):
    """
    A global location object.

    The latitude and longitude are relative to the `WGS84 coordinate system <http://en.wikipedia.org/wiki/World_Geodetic_System>`_.
    The altitude is relative to mean sea-level (MSL).

    For example, a global location object with altitude 30 metres above sea level might be defined as:

    .. code:: python

       LocationGlobal(-34.364114, 149.166022, 30)

    .. todo:: FIXME: Location class - possibly add a vector3 representation.

    An object of this type is owned by :py:attr:`Vehicle.location`. See that class for information on
    reading and observing location in the global frame.

    :param lat: Latitude.
    :param lon: Longitude.
    :param alt: Altitude in meters relative to mean sea-level (MSL).
    """

    def __init__(self, lat, lon, alt=None):
        self.lat = lat
        self.lon = lon
        self.alt = alt

        # This is for backward compatibility.
        self.local_frame = None
        self.global_frame = None

    def __str__(self):
        return "LocationGlobal:lat=%s,lon=%s,alt=%s" % (self.lat, self.lon, self.alt)


class LocationGlobalRelative(object):
    """
    A global location object, with attitude relative to home location altitude.

    The latitude and longitude are relative to the `WGS84 coordinate system <http://en.wikipedia.org/wiki/World_Geodetic_System>`_.
    The altitude is relative to the *home position*.

    For example, a ``LocationGlobalRelative`` object with an altitude of 30 metres above the home location might be defined as:

    .. code:: python

       LocationGlobalRelative(-34.364114, 149.166022, 30)

    .. todo:: FIXME: Location class - possibly add a vector3 representation.

    An object of this type is owned by :py:attr:`Vehicle.location`. See that class for information on
    reading and observing location in the global-relative frame.

    :param lat: Latitude.
    :param lon: Longitude.
    :param alt: Altitude in meters (relative to the home location).
    """

    def __init__(self, lat, lon, alt=None):
        self.lat = lat
        self.lon = lon
        self.alt = alt

        # This is for backward compatibility.
        self.local_frame = None
        self.global_frame = None

    def __str__(self):
        return "LocationGlobalRelative:lat=%s,lon=%s,alt=%s" % (self.lat, self.lon, self.alt)


class LocationLocal(object):
    """
    A local location object.

    The north, east and down are relative to the EKF origin.  This is most likely the location where the vehicle was turned on.

    An object of this type is owned by :py:attr:`Vehicle.location`. See that class for information on
    reading and observing location in the local frame.

    :param north: Position north of the EKF origin in meters.
    :param east: Position east of the EKF origin in meters.
    :param down: Position down from the EKF origin in meters. (i.e. negative altitude in meters)
    """

    def __init__(self, north, east, down):
        self.north = north
        self.east = east
        self.down = down

    def __str__(self):
        return "LocationLocal:north=%s,east=%s,down=%s" % (self.north, self.east, self.down)

    def distance_home(self):
        """
        Distance away from home, in meters. Returns 3D distance if `down` is known, otherwise 2D distance.
        """

        if self.north is not None and self.east is not None:
            if self.down is not None:
                return math.sqrt(self.north**2 + self.east**2 + self.down**2)
            else:
                return math.sqrt(self.north**2 + self.east**2)


class GPSInfo(object):
    """
    Standard information about GPS.

    If there is no GPS lock the parameters are set to ``None``.

    :param Int eph: GPS horizontal dilution of position (HDOP).
    :param Int epv: GPS vertical dilution of position (VDOP).
    :param Int fix_type: 0-1: no fix, 2: 2D fix, 3: 3D fix
    :param Int satellites_visible: Number of satellites visible.

    .. todo:: FIXME: GPSInfo class - possibly normalize eph/epv?  report fix type as string?
    """

    def __init__(self, eph, epv, fix_type, satellites_visible):
        self.eph = eph
        self.epv = epv
        self.fix_type = fix_type
        self.satellites_visible = satellites_visible

    def __str__(self):
        return "GPSInfo:fix=%s,num_sat=%s" % (self.fix_type, self.satellites_visible)


class Wind(object):
    """
    Wind information

    An object of this type is returned by :py:attr: `Vehicle.wind`.

    :param wind_direction: Wind direction in degrees
    :param wind_speed: Wind speed in m/s
    :param wind_speed_z: vertical wind speed in m/s
    """
    def __init__(self, wind_direction, wind_speed, wind_speed_z):
        self.wind_direction = wind_direction
        self.wind_speed = wind_speed
        self.wind_speed_z = wind_speed_z
    
    def __str__(self):
        return "Wind: wind direction: {}, wind speed: {}, wind speed z: {}".format(self.wind_direction, self.wind_speed, self.wind_speed_z)


class Battery(object):
    """
    System battery information.

    An object of this type is returned by :py:attr:`Vehicle.battery`.

    :param voltage: Battery voltage in millivolts.
    :param current: Battery current, in 10 * milliamperes. ``None`` if the autopilot does not support current measurement.
    :param level: Remaining battery energy. ``None`` if the autopilot cannot estimate the remaining battery.
    """

    def __init__(self, voltage, current, level):
        self.voltage = voltage / 1000.0
        if current == -1:
            self.current = None
        else:
            self.current = current / 100.0
        if level == -1:
            self.level = None
        else:
            self.level = level

    def __str__(self):
        return "Battery:voltage={},current={},level={}".format(self.voltage, self.current,
                                                               self.level)


class Rangefinder(object):
    """
    Rangefinder readings.

    An object of this type is returned by :py:attr:`Vehicle.rangefinder`.

    :param distance: Distance (metres). ``None`` if the vehicle doesn't have a rangefinder.
    :param voltage: Voltage (volts). ``None`` if the vehicle doesn't have a rangefinder.
    """

    def __init__(self, distance, voltage):
        self.distance = distance
        self.voltage = voltage

    def __str__(self):
        return "Rangefinder: distance={}, voltage={}".format(self.distance, self.voltage)


class Version(object):
    """
    Autopilot version and type.

    An object of this type is returned by :py:attr:`Vehicle.version`.

    The version number can be read in a few different formats. To get it in a human-readable
    format, just print `vehicle.version`.  This might print something like "APM:Copter-3.3.2-rc4".

    .. versionadded:: 2.0.3

    .. py:attribute:: major

        Major version number (integer).

    .. py:attribute::minor

        Minor version number (integer).

    .. py:attribute:: patch

        Patch version number (integer).

    .. py:attribute:: release

        Release type (integer). See the enum `FIRMWARE_VERSION_TYPE <http://mavlink.org/messages/common#http://mavlink.org/messages/common#FIRMWARE_VERSION_TYPE_DEV>`_.

        This is a composite of the product release cycle stage (rc, beta etc) and the version in that cycle - e.g. 23.

    """
    def __init__(self, raw_version, autopilot_type, vehicle_type):
        self.autopilot_type = autopilot_type
        self.vehicle_type = vehicle_type
        self.raw_version = raw_version
        if raw_version is None:
            self.major = None
            self.minor = None
            self.patch = None
            self.release = None
        else:
            self.major   = raw_version >> 24 & 0xFF
            self.minor   = raw_version >> 16 & 0xFF
            self.patch   = raw_version >> 8  & 0xFF
            self.release = raw_version & 0xFF

    def is_stable(self):
        """
        Returns True if the autopilot reports that the current firmware is a stable
        release (not a pre-release or development version).
        """
        return self.release == 255

    def release_version(self):
        """
        Returns the version within the release type (an integer).
        This method returns "23" for Copter-3.3rc23.
        """
        if self.release is None:
            return None
        if self.release == 255:
            return 0
        return self.release % 64

    def release_type(self):
        """
        Returns text describing the release type e.g. "alpha", "stable" etc.
        """
        if self.release is None:
            return None
        types = ["dev", "alpha", "beta", "rc"]
        return types[self.release >> 6]

    def __str__(self):
        prefix = ""

        if self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            prefix += "APM:"
        elif self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
            prefix += "PX4"
        else:
            prefix += "UnknownAutoPilot"

        if self.vehicle_type == mavutil.mavlink.MAV_TYPE_QUADROTOR:
            prefix += "Copter-"
        elif self.vehicle_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
            prefix += "Plane-"
        elif self.vehicle_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            prefix += "Rover-"
        else:
            prefix += "UnknownVehicleType%d-" % self.vehicle_type

        if self.release_type() is None:
            release_type = "UnknownReleaseType"
        elif self.is_stable():
            release_type = ""
        else:
            # e.g. "-rc23"
            release_type = "-" + str(self.release_type()) + str(self.release_version())

        return prefix + "%s.%s.%s" % (self.major, self.minor, self.patch) + release_type


class Capabilities:
    """
    Autopilot capabilities (supported message types and functionality).

    An object of this type is returned by :py:attr:`Vehicle.capabilities`.

    See the enum
    `MAV_PROTOCOL_CAPABILITY <http://mavlink.org/messages/common#MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT>`_.

    .. versionadded:: 2.0.3


    .. py:attribute:: mission_float

        Autopilot supports MISSION float message type (Boolean).

    .. py:attribute:: param_float

        Autopilot supports the PARAM float message type (Boolean).

    .. py:attribute:: mission_int

        Autopilot supports MISSION_INT scaled integer message type (Boolean).

    .. py:attribute:: command_int

        Autopilot supports COMMAND_INT scaled integer message type (Boolean).

    .. py:attribute:: param_union

        Autopilot supports the PARAM_UNION message type (Boolean).

    .. py:attribute:: ftp

        Autopilot supports ftp for file transfers (Boolean).

    .. py:attribute:: set_attitude_target

        Autopilot supports commanding attitude offboard (Boolean).

    .. py:attribute:: set_attitude_target_local_ned

        Autopilot supports commanding position and velocity targets in local NED frame (Boolean).

    .. py:attribute:: set_altitude_target_global_int

        Autopilot supports commanding position and velocity targets in global scaled integers (Boolean).

    .. py:attribute:: terrain

        Autopilot supports terrain protocol / data handling (Boolean).

    .. py:attribute:: set_actuator_target

        Autopilot supports direct actuator control (Boolean).

    .. py:attribute:: flight_termination

        Autopilot supports the flight termination command (Boolean).

    .. py:attribute:: compass_calibration

        Autopilot supports onboard compass calibration (Boolean).
    """
    def __init__(self, capabilities):
        self.mission_float                  = (((capabilities >> 0)  & 1) == 1)
        self.param_float                    = (((capabilities >> 1)  & 1) == 1)
        self.mission_int                    = (((capabilities >> 2)  & 1) == 1)
        self.command_int                    = (((capabilities >> 3)  & 1) == 1)
        self.param_union                    = (((capabilities >> 4)  & 1) == 1)
        self.ftp                            = (((capabilities >> 5)  & 1) == 1)
        self.set_attitude_target            = (((capabilities >> 6)  & 1) == 1)
        self.set_attitude_target_local_ned  = (((capabilities >> 7)  & 1) == 1)
        self.set_altitude_target_global_int = (((capabilities >> 8)  & 1) == 1)
        self.terrain                        = (((capabilities >> 9)  & 1) == 1)
        self.set_actuator_target            = (((capabilities >> 10) & 1) == 1)
        self.flight_termination             = (((capabilities >> 11) & 1) == 1)
        self.compass_calibration            = (((capabilities >> 12) & 1) == 1)


class VehicleMode(object):
    """
    This object is used to get and set the current "flight mode".

    The flight mode determines the behaviour of the vehicle and what commands it can obey.
    The recommended flight modes for *DroneKit-Python* apps depend on the vehicle type:

    * Copter apps should use ``AUTO`` mode for "normal" waypoint missions and ``GUIDED`` mode otherwise.
    * Plane and Rover apps should use the ``AUTO`` mode in all cases, re-writing the mission commands if "dynamic"
      behaviour is required (they support only a limited subset of commands in ``GUIDED`` mode).
    * Some modes like ``RETURN_TO_LAUNCH`` can be used on all platforms. Care should be taken
      when using manual modes as these may require remote control input from the user.

    The available set of supported flight modes is vehicle-specific (see
    `Copter Modes <http://copter.ardupilot.com/wiki/flying-arducopter/flight-modes/>`_,
    `Plane Modes <http://plane.ardupilot.com/wiki/flying/flight-modes/>`_,
    `Rover Modes <http://rover.ardupilot.com/wiki/configuration-2/#mode_meanings>`_). If an unsupported mode is set the script
    will raise a ``KeyError`` exception.

    The :py:attr:`Vehicle.mode` attribute can be queried for the current mode.
    The code snippet below shows how to observe changes to the mode and then read the value:

    .. code:: python

        #Callback definition for mode observer
        def mode_callback(self, attr_name):
            print "Vehicle Mode", self.mode

        #Add observer callback for attribute `mode`
        vehicle.add_attribute_listener('mode', mode_callback)

    The code snippet below shows how to change the vehicle mode to AUTO:

    .. code:: python

        # Set the vehicle into auto mode
        vehicle.mode = VehicleMode("AUTO")

    For more information on getting/setting/observing the :py:attr:`Vehicle.mode`
    (and other attributes) see the :ref:`attributes guide <vehicle_state_attributes>`.

    .. py:attribute:: name

        The mode name, as a ``string``.
    """

    def __init__(self, name):
        self.name = name

    def __str__(self):
        return "VehicleMode:%s" % self.name

    def __eq__(self, other):
        return self.name == other

    def __ne__(self, other):
        return self.name != other


class SystemStatus(object):
    """
    This object is used to get and set the current "system status".

    An object of this type is returned by :py:attr:`Vehicle.system_status`.

    .. py:attribute:: state

        The system state, as a ``string``.
    """

    def __init__(self, state):
        self.state = state

    def __str__(self):
        return "SystemStatus:%s" % self.state

    def __eq__(self, other):
        return self.state == other

    def __ne__(self, other):
        return self.state != other


class ChannelsOverride(dict):
    """
    A dictionary class for managing Vehicle channel overrides.

    Channels can be read, written, or cleared by index or using a dictionary syntax.
    To clear a value, set it to ``None`` or use ``del`` on the item.

    An object of this type is returned by :py:attr:`Vehicle.channels.overrides <Channels.overrides>`.

    For more information and examples see :ref:`example_channel_overrides`.
    """

    def __init__(self, vehicle):
        self._vehicle = vehicle
        self._count = 8  # Fixed by MAVLink
        self._active = True

    def __getitem__(self, key):
        return dict.__getitem__(self, str(key))

    def __setitem__(self, key, value):
        if not (0 < int(key) <= self._count):
            raise KeyError('Invalid channel index %s' % key)
        if not value:
            try:
                dict.__delitem__(self, str(key))
            except:
                pass
        else:
            dict.__setitem__(self, str(key), value)
        self._send()

    def __delitem__(self, key):
        dict.__delitem__(self, str(key))
        self._send()

    def __len__(self):
        return self._count

    def _send(self):
        if self._active:
            overrides = [0] * 8
            for k, v in self.items():
                overrides[int(k) - 1] = v
            self._vehicle._master.mav.rc_channels_override_send(0, 0, *overrides)


class Channels(dict):
    """
    A dictionary class for managing RC channel information associated with a :py:class:`Vehicle`.

    An object of this type is accessed through :py:attr:`Vehicle.channels`. This object also stores
    the current vehicle channel overrides through its :py:attr:`overrides` attribute.

    For more information and examples see :ref:`example_channel_overrides`.
    """

    def __init__(self, vehicle, count):
        self._vehicle = vehicle
        self._count = count
        self._overrides = ChannelsOverride(vehicle)

        # populate readback
        self._readonly = False
        for k in range(0, count):
            self[k + 1] = None
        self._readonly = True

    @property
    def count(self):
        """
        The number of channels defined in the dictionary (currently 8).
        """
        return self._count

    def __getitem__(self, key):
        return dict.__getitem__(self, str(key))

    def __setitem__(self, key, value):
        if self._readonly:
            raise TypeError('__setitem__ is not supported on Channels object')
        return dict.__setitem__(self, str(key), value)

    def __len__(self):
        return self._count

    def _update_channel(self, channel, value):
        # If we have channels on different ports, we expand the Channels
        # object to support them.
        channel = int(channel)
        self._readonly = False
        self[channel] = value
        self._readonly = True
        self._count = max(self._count, channel)

    @property
    def overrides(self):
        """
        Attribute to read, set and clear channel overrides (also known as "rc overrides")
        associated with a :py:class:`Vehicle` (via :py:class:`Vehicle.channels`). This is an
        object of type :py:class:`ChannelsOverride`.

        For more information and examples see :ref:`example_channel_overrides`.

        To set channel overrides:

        .. code:: python

            # Set and clear overrids using dictionary syntax (clear by setting override to none)
            vehicle.channels.overrides = {'5':None, '6':None,'3':500}

            # You can also set and clear overrides using indexing syntax
            vehicle.channels.overrides['2'] = 200
            vehicle.channels.overrides['2'] = None

            # Clear using 'del'
            del vehicle.channels.overrides['3']

            # Clear all overrides by setting an empty dictionary
            vehicle.channels.overrides = {}

        Read the channel overrides either as a dictionary or by index. Note that you'll get
        a ``KeyError`` exception if you read a channel override that has not been set.

        .. code:: python

            # Get all channel overrides
            print " Channel overrides: %s" % vehicle.channels.overrides
            # Print just one channel override
            print " Ch2 override: %s" % vehicle.channels.overrides['2']
        """
        return self._overrides

    @overrides.setter
    def overrides(self, newch):
        self._overrides._active = False
        self._overrides.clear()
        for k, v in newch.items():
            if v:
                self._overrides[str(k)] = v
            else:
                try:
                    del self._overrides[str(k)]
                except:
                    pass
        self._overrides._active = True
        self._overrides._send()


class Locations(HasObservers):
    """
    An object for holding location information in global, global relative and local frames.

    :py:class:`Vehicle` owns an object of this type. See :py:attr:`Vehicle.location` for information on
    reading and observing location in the different frames.

    The different frames are accessed through the members, which are created with this object.
    They can be read, and are observable.
    """

    def __init__(self, vehicle):
        super(Locations, self).__init__()

        self._lat = None
        self._lon = None
        self._alt = None
        self._relative_alt = None

        @vehicle.on_message('GLOBAL_POSITION_INT')
        def listener(vehicle, name, m):
            (self._lat, self._lon) = (m.lat / 1.0e7, m.lon / 1.0e7)
            self._relative_alt = m.relative_alt / 1000.0
            self.notify_attribute_listeners('global_relative_frame', self.global_relative_frame)
            vehicle.notify_attribute_listeners('location.global_relative_frame',
                                               vehicle.location.global_relative_frame)

            if self._alt is not None or m.alt != 0:
                # Require first alt value to be non-0
                # TODO is this the proper check to do?
                self._alt = m.alt / 1000.0
                self.notify_attribute_listeners('global_frame', self.global_frame)
                vehicle.notify_attribute_listeners('location.global_frame',
                                                   vehicle.location.global_frame)

            vehicle.notify_attribute_listeners('location', vehicle.location)

        self._north = None
        self._east = None
        self._down = None

        @vehicle.on_message('LOCAL_POSITION_NED')
        def listener(vehicle, name, m):
            self._north = m.x
            self._east = m.y
            self._down = m.z
            self.notify_attribute_listeners('local_frame', self.local_frame)
            vehicle.notify_attribute_listeners('location.local_frame', vehicle.location.local_frame)
            vehicle.notify_attribute_listeners('location', vehicle.location)

    @property
    def local_frame(self):
        """
        Location in local NED frame (a :py:class:`LocationGlobalRelative`).

        This is accessed through the :py:attr:`Vehicle.location` attribute:

        .. code-block:: python

            print "Local Location: %s" % vehicle.location.local_frame

        This location will not start to update until the vehicle is armed.
        """
        return LocationLocal(self._north, self._east, self._down)

    @property
    def global_frame(self):
        """
        Location in global frame (a :py:class:`LocationGlobal`).

        The latitude and longitude are relative to the
        `WGS84 coordinate system <http://en.wikipedia.org/wiki/World_Geodetic_System>`_.
        The altitude is relative to mean sea-level (MSL).

        This is accessed through the :py:attr:`Vehicle.location` attribute:

        .. code-block:: python

            print "Global Location: %s" % vehicle.location.global_frame
            print "Sea level altitude is: %s" % vehicle.location.global_frame.alt

        Its ``lat`` and ``lon`` attributes are populated shortly after GPS becomes available.
        The ``alt`` can take several seconds longer to populate (from the barometer).
        Listeners are not notified of changes to this attribute until it has fully populated.

        To watch for changes you can use :py:func:`Vehicle.on_attribute` decorator or
        :py:func:`add_attribute_listener` (decorator approach shown below):

        .. code-block:: python

            @vehicle.on_attribute('location.global_frame')
            def listener(self, attr_name, value):
                print " Global: %s" % value

            #Alternatively, use decorator: ``@vehicle.location.on_attribute('global_frame')``.
        """
        return LocationGlobal(self._lat, self._lon, self._alt)

    @property
    def global_relative_frame(self):
        """
        Location in global frame, with altitude relative to the home location
        (a :py:class:`LocationGlobalRelative`).

        The latitude and longitude are relative to the
        `WGS84 coordinate system <http://en.wikipedia.org/wiki/World_Geodetic_System>`_.
        The altitude is relative to :py:attr:`home location <Vehicle.home_location>`.

        This is accessed through the :py:attr:`Vehicle.location` attribute:

        .. code-block:: python

            print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
            print "Altitude relative to home_location: %s" % vehicle.location.global_relative_frame.alt
        """
        return LocationGlobalRelative(self._lat, self._lon, self._relative_alt)
