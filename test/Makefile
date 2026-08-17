# test/Makefile — iverilog testbench using tb_top.sv
#
# Usage (from repo root):  make -C test
# Usage (from test/):      make
#
# Requires: iverilog (>= 12.0), vvp
#
# The testbench (tb_top.sv) uses $fatal for failures and $finish for pass.
# vvp exits non-zero on $fatal, zero on $finish. results.xml is generated
# so the upstream CI test-summary action still works.

SRC_DIR := ../src
TB      := tb_top.sv
SRCS    := $(SRC_DIR)/compute_core.sv $(SRC_DIR)/interface.sv $(SRC_DIR)/top.sv
VVP     := sim.vvp

.PHONY: all clean

all: results.xml

$(VVP): $(SRCS) $(TB)
	iverilog -g2012 -Wall -Wno-sensitivity-entire-array \
	    -I$(SRC_DIR) -o $@ -s tb_top $(TB) $(SRCS)

results.xml: $(VVP)
	@if vvp $< -fst > sim.log 2>&1; then \
	    cat sim.log; \
	    printf '<?xml version="1.0"?>\n<testsuites><testsuite name="tb_top" tests="1"><testcase name="test_kalman_spi"/></testsuite></testsuites>\n' > $@; \
	    echo "RESULT: PASS"; \
	else \
	    cat sim.log; \
	    printf '<?xml version="1.0"?>\n<testsuites><testsuite name="tb_top" tests="1" failures="1"><testcase name="test_kalman_spi"><failure message="simulation failed — see sim.log"/></testcase></testsuite></testsuites>\n' > $@; \
	    echo "RESULT: FAIL"; \
	    exit 1; \
	fi

clean:
	rm -f $(VVP) sim.log results.xml tb_top.fst
