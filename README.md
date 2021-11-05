# OpenTherm ESP-IDF/ESP8266_RTOS_SDK Library

This library provides implementation of Opentherm protocol.

And this library is ported from Ihor Melnyk's OpenTherm Arduino/ESP8266 Library. I just make it to fit for esp-idf framework. Thank Ihor Melnyk made such a great library.

For the basic usage of this library, please check the example.

OpenTherm protocol requires simple low voltage twowire connection to boiler, but voltage levels (7..15V) still much higher than Arduino/ESP8266 levels, which requires [OpenTherm Adapter](http://ihormelnyk.com/opentherm_adapter).

And for Ihor's Arduino library, Please check[OpenTherm GitHub Link](https://github.com/ihormelnyk/opentherm_library)

In details [OpenTherm Library](http://ihormelnyk.com/opentherm_library) described [here](http://ihormelnyk.com/opentherm_library).