# brewery-controller

A temperature controller for my home brewery. It consists of:
* An ESP32-C3 microcontroller (which is running this repository's code).
* Three 4-digit 7-segment displays, each run by an [HT16K33 datasheet](https://cdn-shop.adafruit.com/datasheets/ht16K33v110.pdf) LED driver over I2C.
* A 3-wire PT1000 RTD (resistance temperature detector).
* A [MAX31865](https://www.analog.com/media/en/technical-documentation/data-sheets/max31865.pdf) RTD resistance-to-digital converter.
* Two [PEC11](https://cdn-shop.adafruit.com/datasheets/pec11.pdf) rotary encoders.

The three 7-segment displays show:
* The RTD's temperature (in degrees Fahrenheit) to one decimal place.
* A "power percent" integer value between 0 and 100, which can be changed using one of the rotary encoders.
* A "target temperature" integer value between 70 and 212, which can be changed using the other rotary encoder.

The binary is no_std, so it runs on the bare metal microcontroller.

| Directory | Description |
| --------- | ----------- |
| [brewery-controller-common](brewery-controller-common) | Library that is agnostic to the underlything hardware, using the [embedded-hal](https://github.com/rust-embedded/embedded-hal) interfaces. |
| [brewery-controller-esp32c3](brewery-controller-esp32c3) | Binary that is specific to the ESP32-C3. |
