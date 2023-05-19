# brewery-controller

A temperature controller for my home brewery. It consists of:
* An ESP32-C3 microcontroller (which is running this repository's code). I use a ESP32-C3-DevKitC-02, but any ESP32-C3 should work if it makes the right pins available.
* Three 4-digit 7-segment displays, each run by an [HT16K33](https://cdn-shop.adafruit.com/datasheets/ht16K33v110.pdf) LED driver.
* A 3-wire PT1000 RTD (resistance temperature detector).
* A [MAX31865](https://www.analog.com/media/en/technical-documentation/data-sheets/max31865.pdf) RTD resistance-to-digital converter.
* Two [PEC11R-4120F-S0018](https://www.bourns.com/docs/product-datasheets/pec11r.pdf) rotary encoders, each on a [Adafruit I2C rotary encoder breakout](https://learn.adafruit.com/adafruit-i2c-qt-rotary-encoder).
* Two two-pole 25A circuit breakers rated for 240 volts AC.
* Two solid state relays (SSR), rated for 40 amps each up to 380 volts AC. They use a control voltage of 3-32 volts DC. Model: [SRDA40-LD](https://www.auberins.com/index.php?main_page=product_info&products_id=980). They are mounted directly to a shared external passive heat sink of dimensions 138mm x 175mm x 30mm. The heat sink is rated for a 60A SSR, which is enough for this application because each 40A SSR is connected to one pole of separate 25A circuit breakers.
* Two NEMA L6-30 sockets, each connected to the output of an SSR. These are wired hot-hot-ground at 240 volts AC with one hot connected to one pole of a 25A circuit breaker and the other to one of the SSRs.

The three 7-segment displays show:
* The RTD's temperature (in degrees Fahrenheit) to one decimal place.
* A "power percent" integer value between 0 and 100, which can be changed using one of the rotary encoders.
* A "target temperature" integer value between 50 and 212, which can be changed using the other rotary encoder.

The displays and the rotary encoders all share the same I2C bus. The MAX31865 is on its own SPI bus and uses an additional interrupt pin to notify the microcontroller when readings are ready. This allows temperature readings at the MAX31865's maximum rate of 60Hz.

The SSRs are controlled using PWM at 3.3V DC, each SSR controlled by its own GPIO pin on the microcontroller. The PWM happens in segments of 100 AC cycles, assuming a 60Hz line frequency. Each cycle corresponds to a percentage point of power (e.g. at 25% power the SSR is on for 25 cycles and off for 75).

The "power percent" directly controls one of the SSRs. The "target temperature" feeds the setpoint of a PID (proportional-integral-derivative) controller whose output is a power percentage for the other SSR.

The binary is no_std, so it runs on the bare metal microcontroller.

| Directory | Description |
| --------- | ----------- |
| [brewery-controller-common](brewery-controller-common) | Library that is agnostic to the underlything hardware, using the [embedded-hal](https://github.com/rust-embedded/embedded-hal) interfaces. |
| [brewery-controller-esp32c3](brewery-controller-esp32c3) | Binary that is specific to the ESP32-C3. |

## Features

The following optional features can be enabled:

| Feature        | Description |
| -------------- | ----------- |
| `server`       | Enables an HTTP server that shows output useful for tuning the PID controller. |
| `calibration`  | Enables "calibration mode", during which the temperature calibration corrections are *disabled* so that the temperature display shows the raw RTD reading. Used to determine the calibration corrections. |

## Server

Enabling the `server` feature causes the WIFI module to connect to a configured access point and listen for HTTP requests.
The server uses DHCP so the WIFI router should configure a static IP for ease of access.

The server is designed to be accessed by a single client. It supports the following requests:

| Request     | Description |
| ----------- | ----------- |
| `GET /`     | Responds with `index.html`, which contains a chart. |
| `GET /data` | Responds with a server-sent events stream, each event containing the next datapoint for the chart. |
| `POST /`    | Accepts new PID tuning values, as provided via the input boxes in index.html. |
