add_library(usb_stm32
	src/usbd_core.c
)

target_include_directories(usb_stm32 PUBLIC
	inc
)

target_link_libraries(usb_stm32
	stm32h7xx_cmsis
)

add_library(usb_stm32h7xx_otghs
	src/usbd_stm32h7xx_otghs.c
)

target_link_libraries(usb_stm32h7xx_otghs
	usb_stm32
)
