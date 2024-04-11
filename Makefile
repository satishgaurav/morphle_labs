


all: upload monitor
upload: upload_pico
monitor: monitor_pico
compile: compile_pico

upload_esp32: 
	@pio run -t upload -e pico

compile_pico: 
	@pio run -e pico 

monitor_pico: 
	@pio device monitor -b 115200

clean: 
	@pio run -t clean

list: 
	@pio device list