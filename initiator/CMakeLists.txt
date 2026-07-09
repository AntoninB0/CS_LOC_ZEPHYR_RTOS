cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(cs_multi_initiator)

target_sources(app PRIVATE
	src/main.c
	src/cs_config.c
	src/beacon.c
	src/pairing.c
	src/cs_ranging.c
	src/distance_estimation.c
	src/ble_protocol_data.c
	src/if.c
	src/scheduler.c
)

target_include_directories(app PRIVATE src)
