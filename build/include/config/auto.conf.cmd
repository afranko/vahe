deps_config := \
	/home/Marci/esp/esp-idf/components/app_trace/Kconfig \
	/home/Marci/esp/esp-idf/components/aws_iot/Kconfig \
	/home/Marci/esp/esp-idf/components/bt/Kconfig \
	/home/Marci/esp/esp-idf/components/driver/Kconfig \
	/home/Marci/esp/esp-idf/components/esp32/Kconfig \
	/home/Marci/esp/esp-idf/components/esp_adc_cal/Kconfig \
	/home/Marci/esp/esp-idf/components/esp_http_client/Kconfig \
	/home/Marci/esp/esp-idf/components/ethernet/Kconfig \
	/home/Marci/esp/esp-idf/components/fatfs/Kconfig \
	/home/Marci/esp/esp-idf/components/freertos/Kconfig \
	/home/Marci/esp/esp-idf/components/heap/Kconfig \
	/home/Marci/esp/esp-idf/components/libsodium/Kconfig \
	/home/Marci/esp/esp-idf/components/log/Kconfig \
	/home/Marci/esp/esp-idf/components/lwip/Kconfig \
	/home/Marci/esp/esp-idf/components/mbedtls/Kconfig \
	/home/Marci/esp/esp-idf/components/mdns/Kconfig \
	/home/Marci/esp/esp-idf/components/openssl/Kconfig \
	/home/Marci/esp/esp-idf/components/pthread/Kconfig \
	/home/Marci/esp/esp-idf/components/spi_flash/Kconfig \
	/home/Marci/esp/esp-idf/components/spiffs/Kconfig \
	/home/Marci/esp/esp-idf/components/tcpip_adapter/Kconfig \
	/home/Marci/esp/esp-idf/components/vfs/Kconfig \
	/home/Marci/esp/esp-idf/components/wear_levelling/Kconfig \
	/home/Marci/esp/esp-idf/Kconfig.compiler \
	/home/Marci/esp/esp-idf/components/bootloader/Kconfig.projbuild \
	/home/Marci/esp/esp-idf/components/esptool_py/Kconfig.projbuild \
	/home/Marci/esp/esp-idf/components/partition_table/Kconfig.projbuild \
	/home/Marci/esp/esp-idf/Kconfig

include/config/auto.conf: \
	$(deps_config)


$(deps_config): ;
