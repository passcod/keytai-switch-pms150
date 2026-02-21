library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;
use std.env.stop;

entity test is
end entity test;

architecture sim of test is

  signal pa_s : std_logic_vector(7 downto 0);

  -- Directly controllable test signals
  signal tb_pa0 : std_logic := 'L';  -- Clock driven by testbench (active-high pulses)
  signal tb_pa5 : std_logic := 'Z';  -- Data driven by testbench during receive phase
  
  -- Mock comparator values
  -- Button circuit: R1=4.7k (BTN1), R2=10k (BTN2), Rgnd=10k
  -- SAR values: none=0, BTN2=7, BTN1=10, both=12
  signal mock_coord_x     : unsigned(3 downto 0) := x"A";  -- Test value = 10, SAR gives 9
  signal mock_coord_y     : unsigned(3 downto 0) := x"5";  -- Test value = 5, SAR gives 4
  signal mock_buttons     : unsigned(3 downto 0) := x"B";  -- 11 -> BTN1 only (SAR 10, quadrant 2)
  
  -- Captured protocol data
  signal captured_x    : std_logic_vector(3 downto 0) := "0000";
  signal captured_y    : std_logic_vector(3 downto 0) := "0000";
  signal captured_btn1 : std_logic := '0';
  signal captured_btn2 : std_logic := '0';
  
  -- Captured capabilities
  signal cap_num_digital  : std_logic_vector(3 downto 0) := "0000";
  signal cap_num_analog   : std_logic_vector(3 downto 0) := "0000";
  signal cap_analog_res_0 : std_logic_vector(3 downto 0) := "0000";
  signal cap_analog_res_1 : std_logic_vector(3 downto 0) := "0000";
  signal cap_num_leds     : std_logic_vector(3 downto 0) := "0000";
  signal cap_led_type_0   : std_logic := '0';
  
  -- Test RGB colour to send (R, G, B)
  constant LED_R_TO_SEND : std_logic_vector(7 downto 0) := x"FF";
  constant LED_G_TO_SEND : std_logic_vector(7 downto 0) := x"80";
  constant LED_B_TO_SEND : std_logic_vector(7 downto 0) := x"20";

  constant CLK_PERIOD : time := 100 us;  -- 10 kHz external clock (slower for DUT response time)
  constant DEBUG_ENABLED: boolean := false;

  -- Helper to read PA5 resolving weak values
  function read_pa5(pa : std_logic_vector) return std_logic is
  begin
    case pa(5) is
      when '1' | 'H' => return '1';
      when '0' | 'L' => return '0';
      when others => return '0';
    end case;
  end function;

begin

  -- DUT instantiation (PMS150C - PDK13, no Port B)
  dut: entity work.pdk13
    generic map (
      DEBUG_ENABLED => DEBUG_ENABLED
    )
    port map (
      PA_io => pa_s,
      -- Mock analog inputs for comparator
      comp_pa7_i => mock_coord_x,   -- PA7 = coord_x
      comp_pa6_i => mock_coord_y,   -- PA6 = coord_y
      comp_pa4_i => mock_buttons    -- PA4 = buttons
    );

  -- Drive PA0 with weak clock signal (testbench is external master)
  -- DUT configures PA0 as input, so it won't fight this
  pa_s(0) <= tb_pa0;
  
  -- Drive PA5 only during receive phase, otherwise high-Z
  pa_s(5) <= tb_pa5;
  
  -- Other PA pins directly from DUT (directly affected by tristate logic)
  pa_s(1) <= 'Z';
  pa_s(2) <= 'Z';
  pa_s(3) <= 'Z';  -- PWM output
  pa_s(4) <= 'Z';
  pa_s(6) <= 'Z';
  pa_s(7) <= 'Z';

  -- PA3 is WS2812 data output (no continuous monitor needed)

  -- Main test process
  stim: process
    variable i : integer;
    variable cap_bit : std_logic;
    variable recv_length : std_logic_vector(7 downto 0);
    
    -- Generate one clock pulse using strong drive (overrides DUT pull-up)
    procedure pulse_clock is
    begin
      tb_pa0 <= '1';
      wait for CLK_PERIOD / 2;
      tb_pa0 <= '0';
      wait for CLK_PERIOD / 2;
    end procedure;
    
    -- Clock high phase with sampling in the middle
    -- DUT sets data BEFORE clock rises, but needs time to process after detecting edge
    -- Sample after DUT has had time to set next bit and stabilize
    procedure clock_high is
    begin
      tb_pa0 <= '1';
      wait for 35 us;  -- Sample after DUT has time to process edge and set data
    end procedure;
    
    -- Complete the clock high phase after sampling
    procedure clock_high_finish is
    begin
      wait for CLK_PERIOD / 2 - 35 us;  -- Remainder of high phase
    end procedure;
    
    procedure clock_low is
    begin
      tb_pa0 <= '0';
      wait for CLK_PERIOD / 2;
    end procedure;
    
    -- Send 24-bit RGB colour (R, G, B) during phase 2 with command 0b01
    procedure send_rgb(r, g, b : std_logic_vector(7 downto 0)) is
    begin
      -- High preamble: gives keyswitch time to switch PA5 to input
      tb_pa5 <= '1';
      pulse_clock;
      pulse_clock;
      -- Send sync pattern: drive PA5 low for 4 clock beats
      tb_pa5 <= '0';
      for i in 0 to 3 loop
        pulse_clock;
      end loop;
      -- Send command 0b01 (colour update)
      tb_pa5 <= '0';
      pulse_clock;
      tb_pa5 <= '1';
      pulse_clock;
      -- Send R (8 bits, MSB first)
      for i in 7 downto 0 loop
        tb_pa5 <= r(i);
        pulse_clock;
      end loop;
      -- Send G (8 bits, MSB first)
      for i in 7 downto 0 loop
        tb_pa5 <= g(i);
        pulse_clock;
      end loop;
      -- Send B (8 bits, MSB first)
      for i in 7 downto 0 loop
        tb_pa5 <= b(i);
        pulse_clock;
      end loop;
      -- Release PA5
      tb_pa5 <= 'Z';
    end procedure;
    
    -- Send skip command (no colour update) during phase 2
    procedure send_skip is
    begin
      -- High preamble: gives keyswitch time to switch PA5 to input
      tb_pa5 <= '1';
      pulse_clock;
      pulse_clock;
      -- Send sync pattern: drive PA5 low for 4 clock beats
      tb_pa5 <= '0';
      for i in 0 to 3 loop
        pulse_clock;
      end loop;
      -- Send command 0b00 (skip)
      tb_pa5 <= '0';
      pulse_clock;
      pulse_clock;
      -- Release PA5
      tb_pa5 <= 'Z';
    end procedure;
    
    -- Send turn-off command (0b11) during phase 2
    procedure send_off is
    begin
      -- High preamble: gives keyswitch time to switch PA5 to input
      tb_pa5 <= '1';
      pulse_clock;
      pulse_clock;
      -- Send sync pattern: drive PA5 low for 4 clock beats
      tb_pa5 <= '0';
      for i in 0 to 3 loop
        pulse_clock;
      end loop;
      -- Send command 0b11 (turn off all LEDs)
      tb_pa5 <= '1';
      pulse_clock;
      pulse_clock;
      -- Release PA5
      tb_pa5 <= 'Z';
    end procedure;
    
    -- Trigger exchange and read 8-bit length prefix from phase 1
    procedure trigger_and_recv_length is
    begin
      -- Trigger: first rising edge, DUT outputs length bit 7
      pulse_clock;
      wait for 50 us;
      -- Read remaining 7 bits (bits 6 downto 0)
      for i in 6 downto 0 loop
        clock_high;
        recv_length(i) := read_pa5(pa_s);
        clock_high_finish;
        clock_low;
      end loop;
      -- Bit 7 is always 0 (lengths < 128)
      recv_length(7) := '0';
    end procedure;
    
  begin
    -- Initialize
    tb_pa0 <= '0';
    tb_pa5 <= 'Z';  -- Let DUT drive during its transmit phase
    
    -- Wait for DUT to initialize (PA5 starts undefined, needs time to stabilize)
    wait for 100 us;
    
    -- =========================================================
    -- CAP TEST: Capability discovery (first exchange)
    -- =========================================================
    report "=== CAP TEST: Capability discovery ===";
    
    -- Wait for DUT to complete first sample cycle and signal ready
    report "Waiting for PA5 ready signal...";
    wait until read_pa5(pa_s) = '1' for 2000 us;
    
    if read_pa5(pa_s) /= '1' then
      report "TIMEOUT waiting for ready signal" severity warning;
      stop;
    end if;
    
    -- Trigger capability exchange and read length prefix
    trigger_and_recv_length;
    report "Received length prefix: " & integer'image(to_integer(unsigned(recv_length)));
    
    -- Receive num_digital (4 bits, MSB first)
    for i in 3 downto 0 loop
      clock_high;
      cap_num_digital(i) <= read_pa5(pa_s);
      clock_high_finish;
      clock_low;
    end loop;
    
    -- Receive num_analog (4 bits, MSB first)
    for i in 3 downto 0 loop
      clock_high;
      cap_num_analog(i) <= read_pa5(pa_s);
      clock_high_finish;
      clock_low;
    end loop;
    
    -- Receive analog_resolution for channel 0 (4 bits, MSB first)
    for i in 3 downto 0 loop
      clock_high;
      cap_analog_res_0(i) <= read_pa5(pa_s);
      clock_high_finish;
      clock_low;
    end loop;
    
    -- Receive analog_resolution for channel 1 (4 bits, MSB first)
    for i in 3 downto 0 loop
      clock_high;
      cap_analog_res_1(i) <= read_pa5(pa_s);
      clock_high_finish;
      clock_low;
    end loop;
    
    -- Receive num_leds (4 bits, MSB first)
    for i in 3 downto 0 loop
      clock_high;
      cap_num_leds(i) <= read_pa5(pa_s);
      clock_high_finish;
      clock_low;
    end loop;
    
    -- Receive led_type for LED 0 (1 bit)
    clock_high;
    cap_led_type_0 <= read_pa5(pa_s);
    clock_high_finish;
    clock_low;
    
    -- Send skip during capability exchange
    send_skip;
    wait for 200 us;
    
    -- Verify capabilities
    report "--- CAP TEST Results ---";
    report "num_digital=" & integer'image(to_integer(unsigned(cap_num_digital)))
         & " num_analog=" & integer'image(to_integer(unsigned(cap_num_analog)))
         & " res_0=" & integer'image(to_integer(unsigned(cap_analog_res_0)))
         & " res_1=" & integer'image(to_integer(unsigned(cap_analog_res_1)))
         & " num_leds=" & integer'image(to_integer(unsigned(cap_num_leds)))
         & " led_type_0=" & std_logic'image(cap_led_type_0);
    
    if unsigned(cap_num_digital) = 2 and unsigned(cap_num_analog) = 2
       and unsigned(cap_analog_res_0) = 4 and unsigned(cap_analog_res_1) = 4
       and unsigned(cap_num_leds) = 1
       and cap_led_type_0 = '1'
       and unsigned(recv_length) = 21 then
      report "CAP TEST PASSED";
    else
      report "CAP TEST FAILED" severity warning;
    end if;
    
    -- =========================================================
    -- TEST 1: Wait for ready signal, then exchange data
    -- =========================================================
    report "=== TEST 1: Wait for PA5 ready signal ===";
    
    -- Wait for DUT to complete first sample cycle and signal ready (PA5 high)
    report "Waiting for PA5 ready signal...";
    wait until read_pa5(pa_s) = '1' for 2000 us;
    
    if read_pa5(pa_s) /= '1' then
      report "TIMEOUT waiting for ready signal" severity warning;
      stop;
    end if;
    
    report "PA5 ready signal detected, triggering exchange";
    
    -- Trigger protocol and read length prefix
    trigger_and_recv_length;
    
    -- Verify ready signal was cleared
    if read_pa5(pa_s) = '1' then
      report "WARNING: Ready signal not cleared after trigger" severity warning;
    end if;
    
    -- Receive coord_x (4 bits, MSB first)
    for i in 3 downto 0 loop
      clock_high;
      captured_x(i) <= read_pa5(pa_s);
      clock_high_finish;
      clock_low;
    end loop;
    report "Captured coord_x: " & integer'image(to_integer(unsigned(captured_x)));
    
    -- Receive coord_y (4 bits, MSB first)
    for i in 3 downto 0 loop
      clock_high;
      captured_y(i) <= read_pa5(pa_s);
      clock_high_finish;
      clock_low;
    end loop;
    report "Captured coord_y: " & integer'image(to_integer(unsigned(captured_y)));
    
    -- Receive btn1
    clock_high;
    captured_btn1 <= read_pa5(pa_s);
    clock_high_finish;
    clock_low;
    
    -- Receive btn2
    clock_high;
    captured_btn2 <= read_pa5(pa_s);
    clock_high_finish;
    clock_low;
    
    report "Captured btn1=" & std_logic'image(captured_btn1) & " btn2=" & std_logic'image(captured_btn2);
    
    -- Now DUT switches PA5 to input mode for reception
    
    -- Send RGB colour (24 bits: R, G, B)
    send_rgb(LED_R_TO_SEND, LED_G_TO_SEND, LED_B_TO_SEND);
    
    -- Wait for DUT to process WS2812 output
    wait for 200 us;
    
    -- Check TEST 1 results
    report "--- TEST 1 Results ---";
    if unsigned(captured_x) = 9 and unsigned(captured_y) = 4 and captured_btn1 = '1' and captured_btn2 = '0' then
      report "TEST 1 PASSED";
    else
      report "TEST 1 FAILED" severity warning;
    end if;
    
    -- =========================================================
    -- TEST 2: Poll without ready signal (no data change)
    -- =========================================================
    report "=== TEST 2: Poll without ready signal (stale data) ===";
    
    -- Mock values haven't changed, so DUT won't set ready flag
    -- Wait briefly to confirm no ready signal
    wait for 500 us;
    
    if read_pa5(pa_s) = '1' then
      report "Unexpected ready signal (values should not have changed)";
    else
      report "No ready signal as expected, polling anyway...";
    end if;
    
    -- Trigger protocol even without ready signal (master polling)
    trigger_and_recv_length;
    
    -- Receive coord_x
    for i in 3 downto 0 loop
      clock_high;
      cap_bit := read_pa5(pa_s);
      captured_x(i) <= cap_bit;
      clock_high_finish;
      clock_low;
    end loop;
    report "Captured coord_x: " & integer'image(to_integer(unsigned(captured_x)));
    
    -- Receive coord_y
    for i in 3 downto 0 loop
      clock_high;
      captured_y(i) <= read_pa5(pa_s);
      clock_high_finish;
      clock_low;
    end loop;
    report "Captured coord_y: " & integer'image(to_integer(unsigned(captured_y)));
    
    -- Receive btn1, btn2
    clock_high;
    captured_btn1 <= read_pa5(pa_s);
    clock_high_finish;
    clock_low;
    clock_high;
    captured_btn2 <= read_pa5(pa_s);
    clock_high_finish;
    clock_low;
    
    report "Captured btn1=" & std_logic'image(captured_btn1) & " btn2=" & std_logic'image(captured_btn2);
    
    -- Complete phase 2 with skip (no colour update)
    send_skip;
    report "TEST 2 skip command sent";
    wait for 500 us;  -- Give DUT time to finish ISR
    
    -- Check TEST 2 results (should get same values as TEST 1 - stale but consistent)
    report "--- TEST 2 Results ---";
    if unsigned(captured_x) = 9 and unsigned(captured_y) = 4 and captured_btn1 = '1' and captured_btn2 = '0' then
      report "TEST 2 PASSED (stale data is consistent)";
    else
      report "TEST 2 FAILED" severity warning;
    end if;
    
    -- =========================================================
    -- TEST 3: Change mock values, verify ready signal fires
    -- =========================================================
    report "=== TEST 3: Change mock values, verify ready signal ===";
    
    -- Change mock values
    mock_coord_x <= x"C";  -- 12, SAR gives 11
    mock_coord_y <= x"3";  -- 3, SAR gives 2
    mock_buttons <= x"8";  -- 8 -> BTN2 only (quadrant 1: 4-7, compare at 8 gives 0, at 4 gives 1)
    
    -- Wait for at least one complete SAR cycle to use new values
    wait for 1000 us;
    
    -- Now wait for DUT to detect change and signal ready
    report "Changed mock values, waiting for ready signal...";
    wait until read_pa5(pa_s) = '1' for 2000 us;
    
    if read_pa5(pa_s) /= '1' then
      report "TIMEOUT waiting for ready signal after value change" severity warning;
      stop;
    end if;
    
    report "Ready signal detected after value change";
    
    -- Exchange with new values
    trigger_and_recv_length;
    
    for i in 3 downto 0 loop
      clock_high;
      captured_x(i) <= read_pa5(pa_s);
      clock_high_finish;
      clock_low;
    end loop;
    report "Captured coord_x: " & integer'image(to_integer(unsigned(captured_x)));
    
    for i in 3 downto 0 loop
      clock_high;
      captured_y(i) <= read_pa5(pa_s);
      clock_high_finish;
      clock_low;
    end loop;
    report "Captured coord_y: " & integer'image(to_integer(unsigned(captured_y)));
    
    clock_high;
    captured_btn1 <= read_pa5(pa_s);
    clock_high_finish;
    clock_low;
    clock_high;
    captured_btn2 <= read_pa5(pa_s);
    clock_high_finish;
    clock_low;
    
    report "Captured btn1=" & std_logic'image(captured_btn1) & " btn2=" & std_logic'image(captured_btn2);
    
    send_off;  -- Command 0b11: turn off all LEDs
    wait for 200 us;
    
    -- Check TEST 3 results
    -- mock_coord_x=12, SAR gives 11
    -- mock_coord_y=3, SAR gives 2
    -- mock_buttons=8: compare at 8 -> false (8 is not > 8), compare at 4 -> true -> BTN2 only
    report "--- TEST 3 Results ---";
    report "Expected coord_x=11 (SAR of 12), got " & integer'image(to_integer(unsigned(captured_x)));
    report "Expected coord_y=2 (SAR of 3), got " & integer'image(to_integer(unsigned(captured_y)));
    report "Expected btn1='0' btn2='1', got btn1=" & std_logic'image(captured_btn1) & " btn2=" & std_logic'image(captured_btn2);
    
    if unsigned(captured_x) = 11 and unsigned(captured_y) = 2 and captured_btn1 = '0' and captured_btn2 = '1' then
      report "TEST 3 PASSED";
    else
      report "TEST 3 FAILED" severity warning;
    end if;
    
    -- =========================================================
    -- Final summary
    -- =========================================================
    report "=== All tests complete ===";
    stop;
  end process;

end sim;
