# whi_temperature_humidity
Temperature and humidity sensor node

## Dependency
```
git clone https://github.com/xinjuezou-whi/whi_interfaces.git
```

## Advertise services
**temperature_humidity**(whi_interfaces::WhiSrvTemperatureHumidity)

Offers the service of requesting the temperature and humidity values, here is an example of a client's request:
```
rosservice call /whi_temperature_humidity/temperature_humidity
```

## Publish topics
**temperature_humidity**(whi_interfaces::WhiSrvTemperatureHumidity)

Publishes the values of temperature and humidity

## Configure params
```
loop_duration: 4.0 # Hz
protocol_config: /home/ros2_ws/catkin_workspace/src/whi_temperature_humidity/config/serial_protocol.yaml
port: /dev/ttyUART_485_2
baudrate: 9600
device_addr: 0x01
```
