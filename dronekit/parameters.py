import collections
import logging
import struct
import time
from time import monotonic

from dronekit.observers import HasObservers

class APIException(Exception):
    """
    Base class for DroneKit related exceptions.

    :param String message: Message string describing the exception
    """

class Parameters(collections.MutableMapping, HasObservers):
    """
    This object is used to get and set the values of named parameters for a vehicle. See the following links for information about
    the supported parameters for each platform: `Copter Parameters <http://copter.ardupilot.com/wiki/configuration/arducopter-parameters/>`_,
    `Plane Parameters <http://plane.ardupilot.com/wiki/arduplane-parameters/>`_, `Rover Parameters <http://rover.ardupilot.com/wiki/apmrover2-parameters/>`_.

    The code fragment below shows how to get and set the value of a parameter.

    .. code:: python

        # Print the value of the THR_MIN parameter.
        print "Param: %s" % vehicle.parameters['THR_MIN']

        # Change the parameter value to something different.
        vehicle.parameters['THR_MIN']=100

    It is also possible to observe parameters and to iterate the :py:attr:`Vehicle.parameters`.

    For more information see :ref:`the guide <vehicle_state_parameters>`.
    """

    def __init__(self, vehicle):
        super(Parameters, self).__init__()
        self._logger = logging.getLogger(__name__)
        self._vehicle = vehicle

    def __getitem__(self, name):
        name = name.upper()
        self.wait_ready()
        return self._vehicle._params_map[name]

    def __setitem__(self, name, value):
        name = name.upper()
        self.wait_ready()
        self.set(name, value)

    def __delitem__(self, name):
        raise APIException('Cannot delete value from parameters list.')

    def __len__(self):
        return len(self._vehicle._params_map)

    def __iter__(self):
        return self._vehicle._params_map.__iter__()

    def get(self, name, wait_ready=True):
        name = name.upper()
        if wait_ready:
            self.wait_ready()
        return self._vehicle._params_map.get(name, None)

    def set(self, name, value, retries=3, wait_ready=False):
        if wait_ready:
            self.wait_ready()

        # TODO dumbly reimplement this using timeout loops
        # because we should actually be awaiting an ACK of PARAM_VALUE
        # changed, but we don't have a proper ack structure, we'll
        # instead just wait until the value itself was changed

        name = name.upper()
        # convert to single precision floating point number (the type used by low level mavlink messages)
        value = float(struct.unpack('f', struct.pack('f', value))[0])
        remaining = retries
        while True:
            self._vehicle._master.param_set_send(name, value)
            tstart = monotonic.monotonic()
            if remaining == 0:
                break
            remaining -= 1
            while monotonic.monotonic() - tstart < 1:
                if name in self._vehicle._params_map and self._vehicle._params_map[name] == value:
                    return True
                time.sleep(0.1)

        if retries > 0:
            self._logger.error("timeout setting parameter %s to %f" % (name, value))
        return False

    def wait_ready(self, **kwargs):
        """
        Block the calling thread until parameters have been downloaded
        """
        self._vehicle.wait_ready('parameters', **kwargs)

    def add_attribute_listener(self, attr_name, *args, **kwargs):
        """
        Add a listener callback on a particular parameter.

        The callback can be removed using :py:func:`remove_attribute_listener`.

        .. note::

            The :py:func:`on_attribute` decorator performs the same operation as this method, but with
            a more elegant syntax. Use ``add_attribute_listener`` only if you will need to remove
            the observer.

        The callback function is invoked only when the parameter changes.

        The callback arguments are:

        * ``self`` - the associated :py:class:`Parameters`.
        * ``attr_name`` - the parameter name. This can be used to infer which parameter has triggered
          if the same callback is used for watching multiple parameters.
        * ``msg`` - the new parameter value (so you don't need to re-query the vehicle object).

        The example below shows how to get callbacks for the ``THR_MIN`` parameter:

        .. code:: python

            #Callback function for the THR_MIN parameter
            def thr_min_callback(self, attr_name, value):
                print " PARAMETER CALLBACK: %s changed to: %s" % (attr_name, value)

            #Add observer for the vehicle's THR_MIN parameter
            vehicle.parameters.add_attribute_listener('THR_MIN', thr_min_callback)

        See :ref:`vehicle_state_observing_parameters` for more information.

        :param String attr_name: The name of the parameter to watch (or '*' to watch all parameters).
        :param args: The callback to invoke when a change in the parameter is detected.

        """
        attr_name = attr_name.upper()
        return super(Parameters, self).add_attribute_listener(attr_name, *args, **kwargs)

    def remove_attribute_listener(self, attr_name, *args, **kwargs):
        """
        Remove a paremeter listener that was previously added using :py:func:`add_attribute_listener`.

        For example to remove the ``thr_min_callback()`` callback function:

        .. code:: python

            vehicle.parameters.remove_attribute_listener('thr_min', thr_min_callback)

        See :ref:`vehicle_state_observing_parameters` for more information.

        :param String attr_name: The parameter name that is to have an observer removed (or '*' to remove an 'all attribute' observer).
        :param args: The callback function to remove.

        """
        attr_name = attr_name.upper()
        return super(Parameters, self).remove_attribute_listener(attr_name, *args, **kwargs)

    def notify_attribute_listeners(self, attr_name, *args, **kwargs):
        attr_name = attr_name.upper()
        return super(Parameters, self).notify_attribute_listeners(attr_name, *args, **kwargs)

    def on_attribute(self, attr_name, *args, **kwargs):
        """
        Decorator for parameter listeners.

        .. note::

            There is no way to remove a listener added with this decorator. Use
            :py:func:`add_attribute_listener` if you need to be able to remove
            the :py:func:`listener <remove_attribute_listener>`.

        The callback function is invoked only when the parameter changes.

        The callback arguments are:

        * ``self`` - the associated :py:class:`Parameters`.
        * ``attr_name`` - the parameter name. This can be used to infer which parameter has triggered
          if the same callback is used for watching multiple parameters.
        * ``msg`` - the new parameter value (so you don't need to re-query the vehicle object).

        The code fragment below shows how to get callbacks for the ``THR_MIN`` parameter:

        .. code:: python

            @vehicle.parameters.on_attribute('THR_MIN')
            def decorated_thr_min_callback(self, attr_name, value):
                print " PARAMETER CALLBACK: %s changed to: %s" % (attr_name, value)

        See :ref:`vehicle_state_observing_parameters` for more information.

        :param String attr_name: The name of the parameter to watch (or '*' to watch all parameters).
        :param args: The callback to invoke when a change in the parameter is detected.

        """
        attr_name = attr_name.upper()
        return super(Parameters, self).on_attribute(attr_name, *args, **kwargs)

