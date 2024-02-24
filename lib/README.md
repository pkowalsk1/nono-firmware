# External Libraries

This directory is designated for project-specific (private) libraries.

Libraries used:

- **hardware_rosc** - Manually forked from the [pico-extras](https://github.com/raspberrypi/pico-extras/tree/master/src/rp2_common/hardware_rosc) repository. This library provides an API for the ring oscillator.

- **pico_sleep** - Also manually forked from the [pico-extras](https://github.com/raspberrypi/pico-extras/tree/master/src/rp2_common/pico_sleep) repository. This library contains APIs related to low power modes. It's a work in progress as it's not sufficiently generic and currently only handles core 0.