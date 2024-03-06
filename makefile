cc = g++   # use g++ compiler
root = ..  # use parent directory for include directives and style

flags = -std=c++17  				# compile with C++ 17 standard
flags += -Wall      				# compile with all warnings
flags += -I $(root) 				# add root to include path
flags += -I /usr/include/modbus  	# add libmodbus to include path
flags += -I /usr/include/yaml-cpp	# add yaml-cpp to include path
flags += -g         				# instrument for gdb


## must set LD_LIBRARY_PATH=/home/user/modbus/lib
## must set LD_RUN_PATH=/home/user/modbus/lib

link = $(cc) $(flags) -o  # final linked build to binary executable

compile = $(cc) $(flags) -c  # compilation to intermediary .o files

bin/logger.o : src/logger.cc inc/logger.h
	$(compile) $< -o $@

## make the main firmware executable
bin/main.o : src/main.cc inc/definitions.h inc/logger.h
	$(compile) $< -o $@

build/optical-engine : bin/main.o bin/logger.o /usr/lib/x86_64-linux-gnu/libmodbus.so /usr/lib/x86_64-linux-gnu/libi2c.a /usr/lib/x86_64-linux-gnu/libyaml-cpp.so
	$(link) $@ $^


## test pinging the modbus server
bin/test_modbus.o : test/test_modbus.cc
	$(compile) $< -o $@

test-modbus : bin/test_modbus.o /usr/lib/x86_64-linux-gnu/libmodbus.so
	$(link) $@ $^


## test the motor write command
bin/test_motor.o : test/test_motor.cc
	$(compile) $< -o $@

test-motor : bin/test_motor.o /usr/lib/x86_64-linux-gnu/libi2c.a
	$(link) $@ $^


## test the serial communications
bin/test_serial.o : test/test_serial.cc
	$(compile) $< -o $@

test-serial : bin/test_serial.o /usr/lib/x86_64-linux-gnu/libi2c.a
	$(link) $@ $^


## make all targets
all : optical-engine test-modbus test-motor test-ping


## clean up resources
clean:
	rm bin/*.o build/*
