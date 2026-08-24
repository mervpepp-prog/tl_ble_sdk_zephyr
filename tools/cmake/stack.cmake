zephyr_include_directories(
	${TL_BLE_SRC_DIR}/stack/ble/hal/hal_${SOC}
	${TL_BLE_SRC_DIR}/stack/system
	${TL_BLE_SRC_DIR}/stack/thread
)

if(CONFIG_BT_TLX_CHANNEL_SOUNDING)
    zephyr_include_directories(${TL_BLE_SRC_DIR}/algorithm/hadm)
    zephyr_link_libraries(${TL_BLE_SRC_DIR}/algorithm/hadm/tlk_algo1/lib/libcs_tlk1_zephyr.a)
endif()


if(CONFIG_SOC_RISCV_TELINK_TL322X AND CONFIG_TELINK_TL322X_ENABLE_N22)
	zephyr_include_directories(${TL_BLE_SRC_DIR}/stack/multicore_comm/service)
endif()