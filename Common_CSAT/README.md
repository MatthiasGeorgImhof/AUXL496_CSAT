Common routines for a distributed STM32 network sharing information with Cyphal
Most code for any particular node should reside in here to be reusable

TestRunner contains Linux tests for all common functionality using HAL mockups as needed

Dependencies:
- [Cyphal](https://opencyphal.org/) with its transmission protocols
    - [LibCanard](https://github.com/OpenCyphal/libcanard) for CAN bus
    - [LibUDPard](https://github.com/OpenCyphal/libudpard) for Ethernet
    - [LibSerard](https://github.com/coderkalyan/libserard/tree/v0-impl) for USB and serial port
    - [Nunavut](https://github.com/OpenCyphal/nunavut)]

    - [Yakut](https://github.com/OpenCyphal/yakut) for monitoring

- [SGP4](https://github.com/CelesTrak/fundamentals-of-astrodynamics/tree/main/software/cpp/SGP4) for orbit calculation

