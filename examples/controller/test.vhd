library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;
use std.env.stop;

entity test is
end entity test;

architecture sim of test is

  signal pa_s : std_logic_vector(7 downto 0);
  signal pb_s : std_logic_vector(7 downto 0);

  -- Directly controllable test signals
  signal tb_pa0 : std_logic := 'L';  -- Clock driven by testbench (active-high pulses)
  signal tb_pa5 : std_logic := 'Z';  -- Data driven by testbench during receive phase
  
  -- Mock comparator values
  signal mock_coord_x     : unsigned(3 downto 0) := x"A";  -- Test value = 10
  signal mock_coord_y     : unsigned(3 downto 0) := x"5";  -- Test value = 5
  signal mock_buttons     : unsigned(3 downto 0) := x"7";  -- btn1=1, btn2=0
  
  -- Captured protocol data
  signal captured_x    : std_logic_vector(3 downto 0) := "0000";
  signal captured_y    : std_logic_vector(3 downto 0) := "0000";
  signal captured_btn1 : std_logic := '0';
  signal captured_btn2 : std_logic := '0';
  
  -- Test LED value to send
  constant LED_TO_SEND : std_logic_vector(7 downto 0) := x"A5";

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

  -- DUT instantiation
  dut: entity work.pdk14
    generic map (
      DEBUG_ENABLED => DEBUG_ENABLED
    )
    port map (
      PA_io => pa_s,
      PB_io => pb_s,
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

  -- Monitor PWM output on PA3
  m1: entity work.measperiod
    generic map (
      NAME => "PWM_PA3"
    )
    port map (
      sig_i => pa_s(3)
    );

  -- Main test process
  stim: process
    variable i : integer;
    variable cap_bit : std_logic;
    
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
    
  begin
    -- Initialize
    tb_pa0 <= '0';
    tb_pa5 <= 'Z';  -- Let DUT drive during its transmit phase
    
    if DEBUG_ENABLED then
      report "pa_s(0) = " & std_logic'image(pa_s(0));
    end if;
    
    -- Let the system initialize and run SAR sampling
    wait for 1200 us;  -- Allow at least one full SAR sample cycle (coord_x, coord_y, buttons) to complete
    
    if DEBUG_ENABLED then
      report "After init, pa_s(0) = " & std_logic'image(pa_s(0));
    end if;
    
    -- Trigger protocol by pulsing PA0 (interrupt on rising edge)
    report "Triggering protocol exchange";
    pulse_clock;
    
    if DEBUG_ENABLED then
      report "After trigger pulse, pa_s(0) = " & std_logic'image(pa_s(0));
    end if;
    
    -- Wait for DUT to enter ISR and set PA5 as output
    wait for 50 us;
    
    -- Clock through the sync pattern (3 more beats - trigger pulse counted as first)
    report "Clocking sync pattern";
    for i in 0 to 2 loop
      pulse_clock;
    end loop;
    
    -- Receive coord_x (4 bits, MSB first)
    for i in 3 downto 0 loop
      clock_high;
      if DEBUG_ENABLED then
        report "Sample coord_x[" & integer'image(i) & "] = pa_s(5)=" & std_logic'image(pa_s(5));
      end if;
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
    -- Wait for DUT to reconfigure
    wait for 20 us;
    
    -- Send sync pattern: drive PA5 low for 4 clock beats
    tb_pa5 <= 'L';
    for i in 0 to 3 loop
      pulse_clock;
    end loop;
    
    -- Send LED value (8 bits, MSB first)
    for i in 7 downto 0 loop
      if LED_TO_SEND(i) = '1' then
        tb_pa5 <= 'H';
      else
        tb_pa5 <= 'L';
      end if;
      pulse_clock;
    end loop;
    
    -- Release PA5
    tb_pa5 <= 'Z';
    
    -- Let DUT process and update PWM
    wait for 100 us;
    
    -- Verify results
    -- Note: SAR uses '>' comparison, so result is 1 less than input when input is exact power of 2
    -- mock_coord_x = 10 (0xA), SAR gives 9 (since 10 is not > 10, final bit test fails)
    -- mock_coord_y = 5 (0x5), SAR gives 4 (since 5 is not > 5, final bit test fails)
    report "=== Test Results ===";
    report "Expected coord_x=9 (SAR of 10), got " & integer'image(to_integer(unsigned(captured_x)));
    report "Expected coord_y=4 (SAR of 5), got " & integer'image(to_integer(unsigned(captured_y)));
    report "Expected btn1='1' btn2='0', got btn1=" & std_logic'image(captured_btn1) & " btn2=" & std_logic'image(captured_btn2);
    
    if unsigned(captured_x) = 9 and unsigned(captured_y) = 4 and captured_btn1 = '1' and captured_btn2 = '0' then
      report "TEST PASSED";
    else
      report "TEST FAILED - values don't match expected" severity warning;
    end if;
    
    report "Test complete";
    stop;
  end process;

end sim;
