NoU_Agent
===============

.. cpp:class:: NoU_Agent

    Handles high level/miscellaneous functions of the NoU3. The library by default provides an object of this class called **NoU3**, The user should never have to create their own :cpp:class:`NoU_Agent` object.`

    **Public Functions**

    .. cpp:function:: begin()

        This function must be called in order for the motor ports to function.

    .. cpp:function:: float getBatteryVoltage()
	
		Gets the Vin voltage supplied to the NoU3. The Vin voltage is divided down with a voltage divider and measured with the internal ADC.
		
		:return: A float representation of the NoU3s Vin voltage.

    .. cpp:function:: float getVersionVoltage()
	
		Gets the version voltage of the NoU3. Each version of the NoU3 has a unique voltage divider that can be measured. Right now there is only one version but this may be useful in the future.
		
		:return: A float representation of the NoU3s version voltage.