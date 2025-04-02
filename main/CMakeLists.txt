idf_component_register(SRCS "main.c" "audio.c"	
					
 INCLUDE_DIRS "." 

 REQUIRES driver nvs_flash spiffs app_update esp_https_ota 
	esp_http_server esp_wifi esp_http_client esp_adc esp_event esp_netif
	esp_lcd usb json esp_jpg fatfs lwip esp-tls esp_websocket_client tcp_transport 
	esp_mdns)

add_prebuilt_library (loco "libloco.a" REQUIRES driver esp_http_client json lwip esp_http_server esp-tls esp_websocket_client esp_netif libhelix esp_mdns)
target_link_libraries (${COMPONENT_LIB} PRIVATE loco)

component_compile_options(-Wno-unused-variable -Wno-error=stringop-overflow)

