TESTBENCH=test
CSOURCE=firmware

GHDL?=ghdl
SDCC?=sdcc
SDCC_CFLAGS=-mpdk13 --opt-code-size --fomit-frame-pointer --fverbose-asm --i-code-in-asm

HDL=../../hdl

COREHDL= $(HDL)/pdk13pkg.vhd \
	$(HDL)/pdkclock.vhd \
	$(HDL)/pdkport.vhd \
	$(HDL)/pdkscaler.vhd \
	$(HDL)/pdktim.vhd \
	$(HDL)/pulsesync.vhd \
	$(HDL)/pdkreg.vhd \
	$(HDL)/txt_util.vhd \
	$(HDL)/measperiod.vhd \
	$(HDL)/pdk13decode.vhd \
	$(HDL)/pdk13.vhd \
	$(HDL)/pdkpkg.vhd

all: $(TESTBENCH)

SOURCES=$(COREHDL) $(TESTBENCH).vhd pdkrom.vhd

$(TESTBENCH): $(SOURCES)
	$(GHDL) -i --std=08 $(SOURCES)
	$(GHDL) -m --std=08 -frelaxed --ieee=synopsys -fexplicit $(TESTBENCH)

pdkrom.vhd: $(CSOURCE).bin
	../../tools/bin2rom -t $(HDL)/pdkrom.vhd.in -i $(CSOURCE).bin -o pdkrom.vhd

$(CSOURCE).bin: $(CSOURCE).ihex
	objcopy -I ihex -O binary $+ $@

$(CSOURCE).ihex: $(CSOURCE).c
	$(SDCC) $(SDCC_CFLAGS) --out-fmt-ihx $+ -o $@

clean:
	rm -f *.o *.rel *.lst *.sym *.asm *.rst *.map *.lk *.ihex *.bin
	rm -f pdkrom.vhd work-obj*.cf $(TESTBENCH) e~*.o *.ghw

.PHONY: all clean
