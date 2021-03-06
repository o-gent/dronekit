import logging

class HasObservers(object):
    def __init__(self):
        logging.basicConfig()
        self._logger = logging.getLogger(__name__)

        # A mapping from attr_name to a list of observers
        self._attribute_listeners = {}
        self._attribute_cache = {}

    def add_attribute_listener(self, attr_name, observer):
        """
        Add an attribute listener callback.

        The callback function (``observer``) is invoked differently depending on the *type of attribute*.
        Attributes that represent sensor values or which are used to monitor connection status are updated
        whenever a message is received from the vehicle. Attributes which reflect vehicle "state" are
        only updated when their values change (for example :py:attr:`Vehicle.system_status`,
        :py:attr:`Vehicle.armed`, and :py:attr:`Vehicle.mode`).

        The callback can be removed using :py:func:`remove_attribute_listener`.

        .. note::

            The :py:func:`on_attribute` decorator performs the same operation as this method, but with
            a more elegant syntax. Use ``add_attribute_listener`` by preference if you will need to remove
            the observer.

        The argument list for the callback is ``observer(object, attr_name, attribute_value)``:

        * ``self`` - the associated :py:class:`Vehicle`. This may be compared to a global vehicle handle
          to implement vehicle-specific callback handling (if needed).
        * ``attr_name`` - the attribute name. This can be used to infer which attribute has triggered
          if the same callback is used for watching several attributes.
        * ``value`` - the attribute value (so you don't need to re-query the vehicle object).

        The example below shows how to get callbacks for (global) location changes:

        .. code:: python

            #Callback to print the location in global frame
            def location_callback(self, attr_name, msg):
                print "Location (Global): ", msg

            #Add observer for the vehicle's current location
            vehicle.add_attribute_listener('global_frame', location_callback)

        See :ref:`vehicle_state_observe_attributes` for more information.

        :param String attr_name: The name of the attribute to watch (or '*' to watch all attributes).
        :param observer: The callback to invoke when a change in the attribute is detected.

        """
        listeners_for_attr = self._attribute_listeners.get(attr_name)
        if listeners_for_attr is None:
            listeners_for_attr = []
            self._attribute_listeners[attr_name] = listeners_for_attr
        if observer not in listeners_for_attr:
            listeners_for_attr.append(observer)

    def remove_attribute_listener(self, attr_name, observer):
        """
        Remove an attribute listener (observer) that was previously added using :py:func:`add_attribute_listener`.

        For example, the following line would remove a previously added vehicle 'global_frame'
        observer called ``location_callback``:

        .. code:: python

            vehicle.remove_attribute_listener('global_frame', location_callback)

        See :ref:`vehicle_state_observe_attributes` for more information.

        :param String attr_name: The attribute name that is to have an observer removed (or '*' to remove an 'all attribute' observer).
        :param observer: The callback function to remove.

        """
        listeners_for_attr = self._attribute_listeners.get(attr_name)
        if listeners_for_attr is not None:
            listeners_for_attr.remove(observer)
            if len(listeners_for_attr) == 0:
                del self._attribute_listeners[attr_name]

    def notify_attribute_listeners(self, attr_name, value, cache=False):
        """
        This method is used to update attribute observers when the named attribute is updated.

        You should call it in your message listeners after updating an attribute with
        information from a vehicle message.

        By default the value of ``cache`` is ``False`` and every update from the vehicle is sent to listeners
        (whether or not the attribute has changed).  This is appropriate for attributes which represent sensor
        or heartbeat-type monitoring.

        Set ``cache=True`` to update listeners only when the value actually changes (cache the previous
        attribute value). This should be used where clients will only ever need to know the value when it has
        changed. For example, this setting has been used for notifying :py:attr:`mode` changes.

        See :ref:`example_create_attribute` for more information.

        :param String attr_name: The name of the attribute that has been updated.
        :param value: The current value of the attribute that has been updated.
        :param Boolean cache: Set ``True`` to only notify observers when the attribute value changes.
        """
        # Cached values are not re-sent if they are unchanged.
        if cache:
            if self._attribute_cache.get(attr_name) == value:
                return
            self._attribute_cache[attr_name] = value

        # Notify observers.
        for fn in self._attribute_listeners.get(attr_name, []):
            try:
                fn(self, attr_name, value)
            except Exception:
                self._logger.exception('Exception in attribute handler for %s' % attr_name, exc_info=True)

        for fn in self._attribute_listeners.get('*', []):
            try:
                fn(self, attr_name, value)
            except Exception:
                self._logger.exception('Exception in attribute handler for %s' % attr_name, exc_info=True)

    def on_attribute(self, name):
        """
        Decorator for attribute listeners.

        The decorated function (``observer``) is invoked differently depending on the *type of attribute*.
        Attributes that represent sensor values or which are used to monitor connection status are updated
        whenever a message is received from the vehicle. Attributes which reflect vehicle "state" are
        only updated when their values change (for example :py:func:`Vehicle.system_status`,
        :py:attr:`Vehicle.armed`, and :py:attr:`Vehicle.mode`).

        The argument list for the callback is ``observer(object, attr_name, attribute_value)``

        * ``self`` - the associated :py:class:`Vehicle`. This may be compared to a global vehicle handle
          to implement vehicle-specific callback handling (if needed).
        * ``attr_name`` - the attribute name. This can be used to infer which attribute has triggered
          if the same callback is used for watching several attributes.
        * ``msg`` - the attribute value (so you don't need to re-query the vehicle object).

        .. note::

            There is no way to remove an attribute listener added with this decorator. Use
            :py:func:`add_attribute_listener` if you need to be able to remove
            the :py:func:`attribute listener <remove_attribute_listener>`.

        The code fragment below shows how you can create a listener for the attitude attribute.

        .. code:: python

            @vehicle.on_attribute('attitude')
            def attitude_listener(self, name, msg):
                print '%s attribute is: %s' % (name, msg)

        See :ref:`vehicle_state_observe_attributes` for more information.

        :param String name: The name of the attribute to watch (or '*' to watch all attributes).
        :param observer: The callback to invoke when a change in the attribute is detected.
        """

        def decorator(fn):
            if isinstance(name, list):
                for n in name:
                    self.add_attribute_listener(n, fn)
            else:
                self.add_attribute_listener(name, fn)

        return decorator