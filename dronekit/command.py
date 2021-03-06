import time

from pymavlink import mavutil


class Command(mavutil.mavlink.MAVLink_mission_item_message):
    """
    A waypoint object.

    This object encodes a single mission item command. The set of commands that are supported
    by ArduPilot in Copter, Plane and Rover (along with their parameters) are listed in the wiki article
    `MAVLink Mission Command Messages (MAV_CMD) <http://planner.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/>`_.

    For example, to create a `NAV_WAYPOINT <http://planner.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_nav_waypoint>`_ command:

    .. code:: python

        cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,-34.364114, 149.166022, 30)

    :param target_system: This can be set to any value
        (DroneKit changes the value to the MAVLink ID of the connected vehicle before the command is sent).
    :param target_component: The component id if the message is intended for a particular component within the target system
        (for example, the camera). Set to zero (broadcast) in most cases.
    :param seq: The sequence number within the mission (the autopilot will reject messages sent out of sequence).
        This should be set to zero as the API will automatically set the correct value when uploading a mission.
    :param frame: The frame of reference used for the location parameters (x, y, z). In most cases this will be
        ``mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT``, which uses the WGS84 global coordinate system for latitude and longitude, but sets altitude
        as relative to the home position in metres (home altitude = 0). For more information `see the wiki here
        <http://planner.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#frames_of_reference>`_.
    :param command: The specific mission command (e.g. ``mavutil.mavlink.MAV_CMD_NAV_WAYPOINT``). The supported commands (and command parameters
        are listed `on the wiki <http://planner.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/>`_.
    :param current: Set to zero (not supported).
    :param autocontinue: Set to zero (not supported).
    :param param1: Command specific parameter (depends on specific `Mission Command (MAV_CMD) <http://planner.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/>`_).
    :param param2: Command specific parameter.
    :param param3: Command specific parameter.
    :param param4: Command specific parameter.
    :param x: (param5) Command specific parameter used for latitude (if relevant to command).
    :param y: (param6) Command specific parameter used for longitude (if relevant to command).
    :param z: (param7) Command specific parameter used for altitude (if relevant). The reference frame for altitude depends on the ``frame``.

    """
    pass


class CommandSequence(object):
    """
    A sequence of vehicle waypoints (a "mission").

    Operations include 'array style' indexed access to the various contained waypoints.

    The current commands/mission for a vehicle are accessed using the :py:attr:`Vehicle.commands` attribute.
    Waypoints are not downloaded from vehicle until :py:func:`download()` is called.  The download is asynchronous;
    use :py:func:`wait_ready()` to block your thread until the download is complete.
    The code to download the commands from a vehicle is shown below:

    .. code-block:: python
        :emphasize-lines: 5-10

        #Connect to a vehicle object (for example, on com14)
        vehicle = connect('com14', wait_ready=True)

        # Download the vehicle waypoints (commands). Wait until download is complete.
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()

    The set of commands can be changed and uploaded to the client. The changes are not guaranteed to be complete until
    :py:func:`upload() <Vehicle.commands.upload>` is called.

    .. code:: python

        cmds = vehicle.commands
        cmds.clear()
        lat = -34.364114,
        lon = 149.166022
        altitude = 30.0
        cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 0, 0, 0, 0, 0,
            lat, lon, altitude)
        cmds.add(cmd)
        cmds.upload()

    """

    def __init__(self, vehicle):
        self._vehicle = vehicle

    def download(self):
        '''
        Download all waypoints from the vehicle.
        The download is asynchronous. Use :py:func:`wait_ready()` to block your thread until the download is complete.
        '''
        self.wait_ready()
        self._vehicle._ready_attrs.remove('commands')
        self._vehicle._wp_loaded = False
        self._vehicle._master.waypoint_request_list_send()
        # BIG FIXME - wait for full wpt download before allowing any of the accessors to work

    def wait_ready(self, **kwargs):
        """
        Block the calling thread until waypoints have been downloaded.

        This can be called after :py:func:`download()` to block the thread until the asynchronous download is complete.
        """
        return self._vehicle.wait_ready('commands', **kwargs)

    def clear(self):
        '''
        Clear the command list.

        This command will be sent to the vehicle only after you call :py:func:`upload() <Vehicle.commands.upload>`.
        '''

        # Add home point again.
        self.wait_ready()
        home = None
        try:
            home = self._vehicle._wploader.wp(0)
        except:
            pass
        self._vehicle._wploader.clear()
        if home:
            self._vehicle._wploader.add(home, comment='Added by DroneKit')
        self._vehicle._wpts_dirty = True

    def add(self, cmd):
        '''
        Add a new command (waypoint) at the end of the command list.

        .. note::

            Commands are sent to the vehicle only after you call ::py:func:`upload() <Vehicle.commands.upload>`.

        :param Command cmd: The command to be added.
        '''
        self.wait_ready()
        self._vehicle._handler.fix_targets(cmd)
        self._vehicle._wploader.add(cmd, comment='Added by DroneKit')
        self._vehicle._wpts_dirty = True

    def upload(self, timeout=None):
        """
        Call ``upload()`` after :py:func:`adding <CommandSequence.add>` or :py:func:`clearing <CommandSequence.clear>` mission commands.

        After the return from ``upload()`` any writes are guaranteed to have completed (or thrown an
        exception) and future reads will see their effects.

        :param int timeout: The timeout for uploading the mission. No timeout if not provided or set to None.
        """
        if self._vehicle._wpts_dirty:
            self._vehicle._master.waypoint_clear_all_send()
            start_time = time.time()
            if self._vehicle._wploader.count() > 0:
                self._vehicle._wp_uploaded = [False] * self._vehicle._wploader.count()
                self._vehicle._master.waypoint_count_send(self._vehicle._wploader.count())
                while False in self._vehicle._wp_uploaded:
                    if timeout and time.time() - start_time > timeout:
                        raise TimeoutError
                    time.sleep(0.1)
                self._vehicle._wp_uploaded = None
            self._vehicle._wpts_dirty = False

    @property
    def count(self):
        '''
        Return number of waypoints.

        :return: The number of waypoints in the sequence.
        '''
        return max(self._vehicle._wploader.count() - 1, 0)

    @property
    def next(self):
        """
        Get the currently active waypoint number.
        """
        return self._vehicle._current_waypoint

    @next.setter
    def next(self, index):
        """
        Set a new ``next`` waypoint for the vehicle.
        """
        self._vehicle._master.waypoint_set_current_send(index)

    def __len__(self):
        '''
        Return number of waypoints.

        :return: The number of waypoints in the sequence.
        '''
        return max(self._vehicle._wploader.count() - 1, 0)

    def __getitem__(self, index):
        if isinstance(index, slice):
            return [self[ii] for ii in range(*index.indices(len(self)))]
        elif isinstance(index, int):
            item = self._vehicle._wploader.wp(index + 1)
            if not item:
                raise IndexError('Index %s out of range.' % index)
            return item
        else:
            raise TypeError('Invalid argument type.')

    def __setitem__(self, index, value):
        self._vehicle._wploader.set(value, index + 1)
        self._vehicle._wpts_dirty = True
