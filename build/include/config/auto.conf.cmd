deps_config := \
	/home/heged/esp/esp-idf/components/app_trace/Kconfig \
	/home/heged/esp/esp-idf/components/aws_iot/Kconfig \
	/home/heged/esp/esp-idf/components/bt/Kconfig \
	/home/heged/esp/esp-idf/components/driver/Kconfig \
	/home/heged/esp/esp-idf/components/esp32/Kconfig \
	/home/heged/esp/esp-idf/components/esp_adc_cal/Kconfig \
	/home/heged/esp/esp-idf/components/esp_http_client/Kconfig \
	/home/heged/esp/esp-idf/components/ethernet/Kconfig \
	/home/heged/esp/esp-idf/components/fatfs/Kconfig \
	/home/heged/esp/esp-idf/components/freertos/Kconfig \
	/home/heged/esp/esp-idf/components/heap/Kconfig \
	/home/heged/esp/esp-idf/components/libsodium/Kconfig \
	/home/heged/esp/esp-idf/components/log/Kconfig \
	/home/heged/esp/esp-idf/components/lwip/Kconfig \
	/home/heged/esp/esp-idf/components/mbedtls/Kconfig \
	/home/heged/esp/esp-idf/components/mdns/Kconfig \
	/home/heged/esp/esp-idf/components/openssl/Kconfig \
	/home/heged/esp/esp-idf/components/pthread/Kconfig \
	/home/heged/esp/esp-idf/components/spi_flash/Kconfig \
	/home/heged/esp/esp-idf/components/spiffs/Kconfig \
	/home/heged/esp/esp-idf/components/tcpip_adapter/Kconfig \
	/home/heged/esp/esp-idf/components/vfs/Kconfig \
	/home/heged/esp/esp-idf/components/wear_levelling/Kconfig \
	/home/heged/esp/esp-idf/Kconfig.compiler \
	/home/heged/esp/esp-idf/components/bootloader/Kconfig.projbuild \
	/home/heged/esp/esp-idf/components/esptool_py/Kconfig.projbuild \
	/home/heged/esp/workspace/aflegacy_vahe/main/Kconfig.projbuild \
	/home/heged/esp/esp-idf/components/partition_table/Kconfig.projbuild \
	/home/heged/esp/esp-idf/Kconfig

include/config/auto.conf: \
	$(deps_config)


$(deps_config): ;
