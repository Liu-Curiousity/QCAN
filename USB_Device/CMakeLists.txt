cmake_minimum_required(VERSION 3.30)

project(USB_Device_Library)

add_library(USB_Device_Library OBJECT)

target_include_directories(USB_Device_Library PUBLIC
        C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.6/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
        ./App
        ./Target
)

target_sources(USB_Device_Library INTERFACE
        ./App/usbd_dfu.c
        ./App/usbd_gs_can.cpp
        ./App/usbd_composite.c
        ./App/usbd_cdc_if.cpp
        ./App/usbd_desc.c
        ./Target/usbd_conf.c
)

target_sources(USB_Device_Library PRIVATE
        C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.6/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
        C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.6/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
        C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_F0_V1.11.6/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
)

target_link_libraries(USB_Device_Library PUBLIC stm32cubemx)


