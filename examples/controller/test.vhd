library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;
use std.env.stop;

entity test is
end entity test;

architecture sim of test is

  signal pa_s : std_logic_vector(7 downto 0);
  signal pb_s : std_logic_vector(7 downto 0);

  -- Protocol signals
  signal clk_ext   : std_logic := '0';  -- External clock on PA0
  signal data_in   : std_logic := '1';  -- Data driven by testbench (when DUT receives)
  
  -- Mock comparator control
  signal mock_comp_result : std_logic := '0';
  signal mock_coord_x     : unsigned(3 downto 0) := x"A";  -- Test value
  signal mock_coord_y     : unsigned(3 downto 0) := x"5";  -- Test value
  signal mock_buttons     : unsigned(3 downto 0) := x"7";  -- btn1=1, btn2=0
  
  -- Captured protocol data
  signal captured_x    : std_logic_vector(3 downto 0);
  signal captured_y    : std_logic_vector(3 downto 0);
  signal captured_btn1 : std_logic;
  signal captured_btn2 : std_logic;
  
  -- Test LED value to send
  signal led_to_send : std_logic_vector(7 downto 0) := x"A5";

  constant CLK_PERIOD : time := 10 us;  -- 100 kHz external clock
  constant DEBUG_ENABLED: boolean := true;

begin

  -- DUT instantiation
  dut: entity work.pdk14
    generic map (
      DEBUG_ENABLED => DEBUG_ENABLED
    )
    port map (
      PA_io => pa_s,
      PB_io => pb_s
    );

  -- Drive PA0 with external clock
  pa_s(0) <= clk_ext;
  
  -- Drive PA5 when testbench is sending (during receive phase)
  -- Otherwise high-Z to let DUT drive
  -- This is simplified: in real sim we'd need proper tristate handling

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
    variable bit_val : std_logic;
    
    -- Generate one clock pulse
    procedure pulse_clock is
    begin
      clk_ext <= '1';
      wait for CLK_PERIOD / 2;
      clk_ext <= '0';
      wait for CLK_PERIOD / 2;
    end procedure;
    
    -- Receive a bit from PA5
    procedure recv_bit(signal b : out std_logic) is
    begin
      clk_ext <= '1';
      wait for CLK_PERIOD / 4;
      b <= pa_s(5);
      wait for CLK_PERIOD / 4;
      clk_ext <= '0';
      wait for CLK_PERIOD / 2;
    end procedure;
    
  begin
    -- Let the system initialize and run SAR sampling for a bit
    wait for 500 us;
    
    -- Trigger protocol by pulsing PA0 (interrupt)
    report "Triggering protocol exchange";
    pulse_clock;
    
    -- Wait for DUT to start sending sync (PA5 low for 4 beats)
    wait for 10 us;
    
    -- Clock through the sync pattern (4 beats)
    for i in 0 to 3 loop
      pulse_clock;
    end loop;
    
    -- Receive coord_x (4 bits)
    for i in 3 downto 0 loop
      clk_ext <= '1';
      wait for CLK_PERIOD / 4;
      captured_x(i) <= pa_s(5);
      wait for CLK_PERIOD / 4;
      clk_ext <= '0';
      wait for CLK_PERIOD / 2;
    end loop;
    report "Captured coord_x: " & integer'image(to_integer(unsigned(captured_x)));
    
    -- Receive coord_y (4 bits)
    for i in 3 downto 0 loop
      clk_ext <= '1';
      wait for CLK_PERIOD / 4;
      captured_y(i) <= pa_s(5);
      wait for CLK_PERIOD / 4;
      clk_ext <= '0';
      wait for CLK_PERIOD / 2;
    end loop;
    report "Captured coord_y: " & integer'image(to_integer(unsigned(captured_y)));
    
    -- Receive btn1, btn2
    clk_ext <= '1';
    wait for CLK_PERIOD / 4;
    captured_btn1 <= pa_s(5);
    wait for CLK_PERIOD / 4;
    clk_ext <= '0';
    wait for CLK_PERIOD / 2;
    
    clk_ext <= '1';
    wait for CLK_PERIOD / 4;
    captured_btn2 <= pa_s(5);
    wait for CLK_PERIOD / 4;
    clk_ext <= '0';
    wait for CLK_PERIOD / 2;
    
    report "Captured btn1=" & std_logic'image(captured_btn1) & " btn2=" & std_logic'image(captured_btn2);
    
    -- Now DUT switches to receive mode
    -- Send sync pattern (hold data low for 4 beats)
    wait for 20 us;  -- Let DUT switch direction
    
    -- Drive sync on PA5 (low for 4 clocks)
    -- Note: This requires tristate handling which is simplified here
    for i in 0 to 3 loop
      pulse_clock;
    end loop;
    
    -- Send LED value (8 bits, MSB first)
    for i in 7 downto 0 loop
      -- Drive data line (would need proper tristate in real sim)
      pulse_clock;
    end loop;
    
    wait for 100 us;
    
    report "Test complete";
    stop;
  end process;

end sim;
