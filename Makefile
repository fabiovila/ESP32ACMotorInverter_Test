PROJECT_NAME := Inverter

IDF_PATH = <PATH_TO_YOUR_ESP_IDF>/esp-idf

PATH  := $(PATH):<PATH_TO_TOOLCHAIN>

SHELL := env PATH=$(PATH) /bin/bash

BTSTACK_ROOT := $(IDF_PATH)/components/btstack/src

include $(IDF_PATH)/make/project.mk

