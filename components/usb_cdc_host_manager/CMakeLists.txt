idf_component_register(
    SRCS
        "src/usb_cdc_host_manager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES usb usb_host_vcp usb_host_cp210x_vcp usb_host_ftdi_vcp usb_host_ch34x_vcp
)
