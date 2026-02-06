-- PMS150C (PDK13) simulation testbench
library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;
use std.env.stop;

entity test is
end entity;

architecture sim of test is
  signal PA_s : std_logic_vector(7 downto 0);
  
  constant DEBUG_ENABLED: boolean := false;
begin

  uut: entity work.pdk13
    generic map (
      DEBUG_ENABLED => DEBUG_ENABLED
    )
    port map (
      PA_io => PA_s,
      eosc_i => 'X',
      comp_pa4_i => x"0",
      comp_pa6_i => x"0", 
      comp_pa7_i => x"0"
    );

  -- Measurement for PWM on PA3
  meas_pa3: entity work.measperiod
    generic map ( NAME => "PA3" )
    port map ( sig_i => PA_s(3) );

  process
  begin
    wait for 500 us;
    report "Ending simulation";
    stop;
  end process;

end sim;
