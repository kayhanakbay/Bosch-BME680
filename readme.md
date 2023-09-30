# Bosch BME680 Sensor For ARM-V7 Static Compile

## BME680 Example For ARM-V7 With CMakeLists && CLion

### BME680

Using the BME680 to measure temperature, pressure, humidity and air quality.

The sensor is used to obtain the so-called Gas Resistance and then calculate an Index of Air Quality (IAQ) from a combination of humidity and the gas content readings of the air, optionally temperature could be added too, but omittied in this example.

The index is a function of humidity which contributes up to 25% and gas concentrations that contributes up to 75%. See slide1.jpg for details of the IAQ index formulation.

In this current version only Humidity and Gas concentrations are used for the index, but adding temperature would be straightforward on the basis that temperatures for humans that are too high or low add to the overal Air Quality index along with Humidity and Gas concetrations.

Humidity is measured between 0 - 100% and is universally accepted as being optimal when it is 40% and where in this index the contribution will be 0, but at a Humidity reading of 0%, the contribution increases to 25% and similarly when humidity reaches 100% it contributes 25% to the index. See Slide1 for details.

Gas concentrations for normal breathable air with no pollutants (no adverse gases) corresponds to the sensors highest resitance output of 50,000 ohms or more. The sensor normally outputs a Gas resistance value ranging from a low of 50ohm to a high of 50,000ohm and beyond. A linear relationship is assumed and the output scaled accordingly between 0 and 75% for the range 50-50,000 ohms.

The result of combining humidity and gas measurements into an in index is a qualitative and so-called IAQ - Indoor Air Quality index value scaled from 0-100% (where 100% is good). This is then scaled again from 0-500 where a 500 value is bad and descriptive values are applied in stages from good to hazardous air quality.

There is no definitive (ISO Standard) method for calculating an IAQ.


### Features

- Air quality measurement
- Personalized weather station
- Context awareness, e.g. skin moisture detection, room change detection
- Fitness monitoring / well-being
- Warning regarding dryness or high temperatures
- Measurement of volume and air flow
- Home automation control (e.g. HVAC)
- GPS enhancement (e.g. time-to-first-fix improvement, dead reckoning, slope detection)
- Indoor navigation (change of floor detection, elevator detection)
- Altitude tracking and calories expenditure for sports activities

#### Important links:
For more information, please refer to:

- [BME680 Product page](https://www.bosch-sensortec.com/bst/products/all_products/bme680)
- [BME680 & BME688 Github page](https://github.com/BoschSensortec/BME68x-Sensor-API)
- [BME680 gas sensor design guide](https://community.bosch-sensortec.com/t5/Knowledge-base/BME680-gas-sensor-series-design-guide/ta-p/5952)
- [Knowledge base page](https://community.bosch-sensortec.com/t5/Knowledge-base/tkb-p/bst_community-mems-tkb)
- [Community support page](https://community.bosch-sensortec.com)

