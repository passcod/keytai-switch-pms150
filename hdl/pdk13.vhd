--
--  FPPA PDK13 Microcontroller simulation model (PMS150C)
--
--  Based on PDK14 model by Alvaro Lopes <alvieboy@alvie.com>
--  Simplified for PMS150C: No Port B, No TM3, No PWMG
--
--  The FreeBSD license
--
--  Redistribution and use in source and binary forms, with or without
--  modification, are permitted provided that the following conditions
--  are met:
--
--  1. Redistributions of source code must retain the above copyright
--     notice, this list of conditions and the following disclaimer.
--  2. Redistributions in binary form must reproduce the above
--     copyright notice, this list of conditions and the following
--     disclaimer in the documentation and/or other materials
--     provided with the distribution.
--
--  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY
--  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
--  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
--  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
--  ZPU PROJECT OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
--  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
--  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
--  OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
--  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
--  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
--  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
--  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--
--

library IEEE;
use IEEE.STD_LOGIC_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_misc.all;

library work;
use work.txt_util.all;
use work.pdk13pkg.all;

entity pdk13 is
  generic (
    -- Setting this to true will enable EOSC input and disable PA6/PA7
    EOSC_CONNECTED: boolean := false;
    DEBUG_ENABLED: boolean := false
  );
  port (
    PA_io   : inout std_logic_vector(7 downto 0);
    -- Special pins with analog behaviour.
    eosc_i  : in std_logic := 'X'; -- External oscillator (not crystal).
    -- Analog inputs for comparator simulation (4-bit value representing voltage level 0-15)
    comp_pa4_i : in unsigned(3 downto 0) := x"0";  -- Analog value on PA4
    comp_pa6_i : in unsigned(3 downto 0) := x"0";  -- Analog value on PA6
    comp_pa7_i : in unsigned(3 downto 0) := x"0"   -- Analog value on PA7
  );
end entity pdk13;

architecture sim of pdk13 is

  --
  -- Local types
  --

  type mem_type is array(0 to 63) of wordtype;  -- PMS150C has 64 bytes RAM

  --
  -- Constants
  --

  constant SFR_INDEX_SP   : natural := 2;

  --
  -- Shared variables
  --

  shared variable mem     : mem_type;
  shared variable tim16set: std_logic := '0';

  --
  -- Signals and registers
  --

  signal opcode_s         : opcodetype;
  signal flags            : flags_type;
  signal A                : wordtype;
  signal ginten           : std_logic;
  signal jmp              : std_logic;
  signal dbg_mem          : mem_type;
  signal opcode_valid     : std_logic := '0';
  signal rst_s            : std_ulogic := '0';
  signal sysclk_s         : std_ulogic := '0';
  signal ihrc_s           : std_logic;
  signal ilrc_s           : std_logic;
  signal eosc_s           : std_logic;
  signal PC               : pctype;
  signal IPC              : pctype;
  signal npc              : pctype;
  signal cycle            : unsigned(31 downto 0):=(others => '0');
  signal dec              : opdec_type;
  signal opcode_full_s    : std_logic_vector(15 downto 0);
  signal int_s            : std_logic := '0';
  signal tim16setvalue    : unsigned(15 downto 0);
  signal timer16clk       : std_ulogic;
  signal t16cnt           : unsigned(15 downto 0);
  signal pt16cnt          : unsigned(15 downto 0);
  signal intsrc_s         : std_logic_vector(7 downto 0);
  signal sfr_read         : wordtype;
  signal sfr_wdata        : wordtype;
  signal sfr_wmask        : wordtype;
  signal sfr_wen          : std_logic_vector(31 downto 0);  -- Only 32 IO registers

  signal SP_s             : wordtype;
  signal CLKMD_s          : wordtype;
  signal INTEN_s          : wordtype;
  signal INTRQ            : wordtype;
  signal T16M_s           : wordtype;
  signal INTEGS_s         : wordtype;
  signal PADIER_s         : wordtype;
  signal PA_s             : wordtype;
  signal PAC_s            : wordtype;
  signal PAPH_s           : wordtype;
  signal MISC_s           : wordtype;
  signal TM2B_s           : wordtype;
  signal TM2C_s           : wordtype;
  signal TM2CT_s          : wordtype;
  signal TM2S_s           : wordtype;
  signal GPCC             : wordtype;
  signal gpcc_reg_s       : wordtype;  -- GPCC register value (without comparison result)
  signal GPCS             : wordtype;

  signal t16intr          : std_logic;
  signal intreq_s         : std_logic_vector(7 downto 0) := (others => '0'); -- Interrupt request
  signal pa0_fall_s       : std_logic;
  signal pa0_rise_s       : std_logic;
  signal stall_s          : std_logic;
  signal tim2out_s        : std_logic;
  signal tim2int_s        : std_logic;
  signal PA_i_s           : std_logic_vector(7 downto 0);
  signal PA_o_s           : std_logic_vector(7 downto 0);

  --
  -- Function and procedure declarations
  --

  function signextend10(a: in wordtype) return unsigned is
    variable r: unsigned(9 downto 0);  -- 10-bit PC for PDK13
  begin
    r(7 downto 0) := a;
    r(9 downto 8) := (others => a(7));
    return r;
  end signextend10;
  
  function neg(a: in wordtype) return unsigned is
    variable r: wordtype;
  begin
    r := not a;
    r := r + 1;
     return r;
  end function;

  procedure writeMem(address: in natural; data: in wordtype) is
  begin
    if DEBUG_ENABLED then
      report " WRITE address=" & str(address) & " value 0x" & hstr(std_logic_vector(data));
    end if;
    mem(address) := data;
  end procedure;

  impure function readMem(address: in natural) return wordtype is
    variable d: wordtype;
  begin
    if (address>mem'HIGH) then
      report "Out of bounds mem access " &str(address) & " pc 0x" & hstr(std_logic_vector(ipc));
    end if;
    d:=mem(address);
    if DEBUG_ENABLED then
      report " READ address=" & str(address) & " value 0x" & hstr(std_logic_vector(d));
    end if;
    return d;
  end function;


begin

  rst_s <= '0', '1' after 10 ps, '0' after 200 ns;

  clock_inst: entity work.pdkclock
  generic map (
    EOSC_CONNECTED => EOSC_CONNECTED
  )
  port map (
    rst_i     => rst_s,
    eosc_i    => eosc_i,
    clkmd_i   => CLKMD_s,
    sysclk_o  => sysclk_s,
    ihrc_o    => ihrc_s,
    ilrc_o    => ilrc_s,
    eosc_o    => eosc_s
  );

  -- Debugging purposes only, to be visible on waveform
  process(sysclk_s)
  begin
    dbg_mem <= mem;
  end process;

  SPinst: entity work.pdkreg generic map ( NAME => "SP" )
    port map ( clk_i => sysclk_s, rst_i => rst_s, dat_i => sfr_wdata, mask_i => sfr_wmask, wen_i => sfr_wen(2), dat_o => SP_s );

  INTENinst: entity work.pdkreg generic map ( NAME => "INTEN" )
    port map ( clk_i => sysclk_s, rst_i => rst_s, dat_i => sfr_wdata, mask_i => sfr_wmask, wen_i => sfr_wen(4), dat_o => INTEN_s );

  INTEGSinst: entity work.pdkreg generic map ( NAME => "INTEGS" )
    port map ( clk_i => sysclk_s, rst_i => rst_s, dat_i => sfr_wdata, mask_i => sfr_wmask, wen_i => sfr_wen(12), dat_o => INTEGS_s );

  PADIERinst: entity work.pdkreg generic map ( NAME => "PADIER" )
    port map ( clk_i => sysclk_s, rst_i => rst_s, dat_i => sfr_wdata, mask_i => sfr_wmask, wen_i => sfr_wen(13), dat_o => PADIER_s );

  T16Minst: entity work.pdkreg generic map ( NAME => "T16M" )
    port map ( clk_i => sysclk_s, rst_i => rst_s, dat_i => sfr_wdata, mask_i => sfr_wmask, wen_i => sfr_wen(6), dat_o => T16M_s );

  CLKMDinst: entity work.pdkreg generic map ( NAME => "CLKMD", RESET => x"E0" )
    port map ( clk_i => sysclk_s, rst_i => rst_s, dat_i => sfr_wdata, mask_i => sfr_wmask, wen_i => sfr_wen(3), dat_o => CLKMD_s );

  MISCinst: entity work.pdkreg generic map ( NAME => "MISC" )
    port map ( clk_i => sysclk_s, rst_i => rst_s, dat_i => sfr_wdata, mask_i => sfr_wmask, wen_i => sfr_wen(8), dat_o => MISC_s );

  -- Comparator registers
  GPCSinst: entity work.pdkreg generic map ( NAME => "GPCS" )
    port map ( clk_i => sysclk_s, rst_i => rst_s, dat_i => sfr_wdata, mask_i => sfr_wmask, wen_i => sfr_wen(25), dat_o => GPCS );

  -- GPCC register (bits 7,6,5 and 3:0 are writable, bit 4 is read-only comparison result)
  GPCCinst: entity work.pdkreg generic map ( NAME => "GPCC" )
    port map ( clk_i => sysclk_s, rst_i => rst_s, dat_i => sfr_wdata, mask_i => sfr_wmask, wen_i => sfr_wen(24), dat_o => gpcc_reg_s );

  -- Combinational comparator logic
  gpcc_cmp: process(all)
    variable comp_input : unsigned(3 downto 0);
    variable comp_ref : unsigned(3 downto 0);
    variable comp_result : std_logic;
  begin
    -- Select comparator input based on GPCS
    case to_integer(GPCS(2 downto 0)) is
      when 1 => comp_input := comp_pa7_i;      -- PA7
      when 2 => comp_input := comp_pa6_i;      -- PA6
      when 3 => comp_input := comp_pa4_i;      -- PA4
      when others => comp_input := x"0";
    end case;
    
    -- Reference level from GPCC bits 3:0
    comp_ref := unsigned(gpcc_reg_s(3 downto 0));
    
    -- Comparator result: 1 if input > reference
    if comp_input > comp_ref then
      comp_result := '1';
    else
      comp_result := '0';
    end if;
    
    -- Debug output
    if DEBUG_ENABLED and gpcc_reg_s(7) = '1' then
      report "COMP: GPCS=" & integer'image(to_integer(GPCS(2 downto 0))) &
             " input=" & integer'image(to_integer(comp_input)) &
             " ref=" & integer'image(to_integer(comp_ref)) &
             " result=" & std_logic'image(comp_result);
    end if;
    
    -- GPCC output: combine register with comparison result in bit 4
    GPCC <= gpcc_reg_s(7 downto 5) & comp_result & gpcc_reg_s(3 downto 0);
  end process;


  -- Port A
  PAinst: entity work.pdkport
    generic map (
      NAME => "PA"
    )
    port map (
    clk_i     => sysclk_s,
    rst_i     => rst_s,
    dat_i     => sfr_wdata,
    mask_i    => sfr_wmask,
    datwen_i  => sfr_wen(16),
    ctrlwen_i => sfr_wen(17),
    pullwen_i => sfr_wen(18),
    i0_rise_o => pa0_rise_s,
    i0_fall_o => pa0_fall_s,
    ctrl_o    => PAC_s,
    pull_o    => PAPH_s,
    pdata_o   => PA_s,
    pad_i     => PA_i_s,
    pad_o     => PA_o_s
  );

  -- Tim2
  tim2inst: entity work.pdktim
    generic map ( NAME => "TIM2" )
    port map (
      clk_i     => sysclk_s,
      rst_i     => rst_s,
      ihrc_i    => ihrc_s,
      eosc_i    => eosc_s,
      ilrc_i    => ilrc_s,
      comp_i    => '0',
      pa0_i     => PA_i_s(0),
      pb0_i     => '0',  -- No Port B on PMS150C
      pa4_i     => PA_i_s(4),
  
      dat_i     => sfr_wdata,
      mask_i    => sfr_wmask,
      c_wen_i   => sfr_wen(28),
      s_wen_i   => sfr_wen(23),
      ct_wen_i  => sfr_wen(29),
      b_wen_i   => sfr_wen(9),
  
      c_o       => TM2C_s,
      s_o       => TM2S_s,
      ct_o      => TM2CT_s,
      b_o       => TM2B_s,
      timout_o  => tim2out_s,
      intr_o    => tim2int_s
  );
  intreq_s(6) <= tim2int_s;


  -- PA0 interrupt
  process(padier_s(0), integs_s(1 downto 0), pa0_rise_s, pa0_fall_s)
  begin
    if padier_s(0)='1' then
      case integs_s(1 downto 0) is
        when "00" => intreq_s(0) <= pa0_rise_s OR pa0_fall_s;
        when "01" => intreq_s(0) <= pa0_rise_s;
        when "10" => intreq_s(0) <= pa0_fall_s;
        when "11" => intreq_s(0) <= '0';
        when others => intreq_s(0) <= '0';
      end case;
    else
      -- Disabled interrupt
      intreq_s(0) <= '0';
    end if;
  end process;


  process( sysclk_s, rst_s )
  begin
    if rst_s='1' then
      INTRQ <= (others => '0');
    elsif rising_edge(sysclk_s) then
      if sfr_wen(5)='1' then
        if DEBUG_ENABLED then
          report "SFR INTRQ write, data " & hstr(std_logic_vector(sfr_wdata)) & " mask " & hstr(std_logic_vector(sfr_wmask));
        end if;
        INTRQ <= (INTRQ and not sfr_wmask) or (sfr_wdata AND sfr_wmask);
      end if;
      -- Set interrupt even if we cleared from SW
      l: for i in 0 to 7 loop
        if intreq_s(i)='1' then
          INTRQ(i) <= '1';
        end if;
      end loop;

    end if;
  end process;


  -- PMS150C SFR map (only 32 IO registers, 5-bit address)
  with to_integer(dec.ioaddr)
    select sfr_read <=
      "0000" & flags.O & flags.AC & flags.C & flags.Z when 0,  -- FLAG
      SP_s        when 2,   -- SP
      CLKMD_s     when 3,   -- CLKMD
      INTEN_s     when 4,   -- INTEN
      INTRQ       when 5,   -- INTRQ
      T16M_s      when 6,   -- T16M
      MISC_s      when 8,   -- MISC
      TM2B_s      when 9,   -- TM2B
      x"00"       when 10,  -- EOSCR (write-only)
      INTEGS_s    when 12,  -- INTEGS
      PADIER_s    when 13,  -- PADIER
      PA_s        when 16,  -- PA
      PAC_s       when 17,  -- PAC
      PAPH_s      when 18,  -- PAPH
      TM2S_s      when 23,  -- TM2S
      GPCC        when 24,  -- GPCC
      GPCS        when 25,  -- GPCS
      TM2C_s      when 28,  -- TM2C
      TM2CT_s     when 29,  -- TM2CT
      (others => 'X') when others;

  -- ROM

  rom: entity work.pdkrom
  port map (
    PC_i => to_integer(pc),
    dat_o => opcode_full_s
  );


  -- Instruction fetcher
  process(sysclk_s,rst_s)
  begin
    if rst_s='1' then
       opcode_valid <= '0';
       PC <= (others => '0');
    elsif rising_edge(sysclk_s) then
      if stall_s='0' then
        if jmp='1' then
          PC <= npc;
          IPC <= npc; -- To avoid interrupt not picking up return address.
          opcode_valid <= '0';
          if DEBUG_ENABLED then
            report "Jumping to " & hstr(std_logic_vector(npc));
          end if;
        else 
          opcode_s <= opcode_full_s(opcode_s'HIGH downto 0);
          opcode_valid <= '1';
          PC <= PC + 1;
          IPC <= PC;
        end if;
      end if;
    end if;
  end process;

  -- Instruction decoder (PDK13)
  decoder_i0: entity work.pdk13decode
  port map (
    opcode_i    => opcode_s,
    valid_i     => opcode_valid,
    decoded_o   => dec
  );

  -- interrupt generation
  intsrc_s  <= std_logic_vector(INTEN_s) and std_logic_vector(INTRQ);
  int_s     <=  or_reduce(intsrc_s);


  -- Jump decoder
  process(dec.decoded, A, IPC, dec.jmpaddr, dec.memaddr, int_s, ginten, sfr_read)
    variable temp: unsigned(7 downto 0);
  begin
    jmp <= '0';
    if int_s='1' and ginten='1' then
      npc <= "00" & x"10";  -- Interrupt vector at 0x10 (10-bit PC)
      jmp <= '1';
    else
      case dec.decoded is
        when opcode_pcadda =>
          npc <= IPC + signextend10(A);
          jmp <= '1';
        when opcode_ret | opcode_reti | opcode_retk =>
          temp := readMem(to_integer(SP_s-1));
          npc(npc'HIGH downto 8) <= temp(1 downto 0);  -- 10-bit PC: bits 9:8
          temp := readMem(to_integer(SP_s-2));
          npc(7 downto 0) <= temp;
          jmp <= '1';
        when opcode_gotok | opcode_callk =>
          npc <= dec.jmpaddr;
          jmp <= '1';
        when opcode_ceqsnam =>  
          if A = readMem( to_integer(dec.memaddr) ) then
            npc <= IPC + 2;
            jmp <= '1';
          end if;
        when opcode_ceqsnak =>  
          if A = dec.immed then
            npc <= IPC + 2;
            jmp <= '1';
          end if;

        when opcode_t0snio =>
          if sfr_read( to_integer(dec.bitaddr) )='0' then
            npc <= IPC + 2;
            jmp <= '1';
          end if;

        when opcode_t0snm =>
          if readMem( to_integer(dec.memaddr) )( to_integer(dec.bitaddr) )='0' then
            npc <= IPC + 2;
            jmp <= '1';
          end if;

        when opcode_t1snio =>
          if sfr_read( to_integer(dec.bitaddr) )='1' then
            npc <= IPC + 2;
            jmp <= '1';
          end if;
          
        when opcode_t1snm =>
          if readMem( to_integer(dec.memaddr) )( to_integer(dec.bitaddr) )='1' then
            npc <= IPC + 2;
            jmp <= '1';
          end if;

        when opcode_izsna =>
          if (A + 1) = 0 then
            npc <= IPC + 2;
            jmp <= '1';
          end if;

        when opcode_dzsna =>
          if (A - 1) = 0 then
            npc <= IPC + 2;
            jmp <= '1';
          end if;

        when opcode_izsnm =>
          temp := readMem( to_integer(dec.memaddr) ) + 1;
          if temp = 0 then
            npc <= IPC + 2;
            jmp <= '1';
          end if;

        when opcode_dzsnm =>
          temp := readMem( to_integer(dec.memaddr) ) - 1;
          if temp = 0 then
            npc <= IPC + 2;
            jmp <= '1';
          end if;
  
        when others => 
      end case;
    end if;
  end process;


  -- SFR write
  process(dec.decoded, sfr_read, dec.ioaddr, dec.bitaddr, opcode_valid)
  begin
    sfr_wen <= (others => '0');
    sfr_wmask <= (others => '1');
    
    if int_s='1' and ginten='1' then
      sfr_wen(SFR_INDEX_SP)<='1';
      sfr_wdata <= SP_s + 2;
    else
      case dec.decoded is 
        when opcode_pushaf  =>  sfr_wen(SFR_INDEX_SP)<='1';
                                sfr_wdata <= SP_s + 2;

        when opcode_popaf   =>  sfr_wen(SFR_INDEX_SP)<='1';
                                sfr_wdata <= SP_s - 2;

        when opcode_ret     =>  sfr_wen(SFR_INDEX_SP)<='1';
                                sfr_wdata <= SP_s - 2;

        when opcode_reti    =>  sfr_wen(SFR_INDEX_SP)<='1';
                                sfr_wdata <= SP_s - 2;

        when opcode_retk    =>  sfr_wen(SFR_INDEX_SP)<='1';
                                sfr_wdata <= SP_s - 2;
 
        when opcode_xorioa  =>  sfr_wdata <= A xor sfr_read;
                                sfr_wen( to_integer(dec.ioaddr) ) <= '1';

        when opcode_movioa  =>  sfr_wdata <= A;
                                sfr_wen( to_integer(dec.ioaddr) ) <= '1';
 
        when opcode_set0io  =>  sfr_wen( to_integer(dec.ioaddr) ) <= '1';
                                sfr_wmask <= (others => '0');
                                sfr_wmask(to_integer(dec.bitaddr)) <= '1';
                                sfr_wdata <= (others => '0');

        when opcode_set1io  =>  sfr_wen( to_integer(dec.ioaddr) ) <= '1';
                                sfr_wmask <= (others => '0');
                                sfr_wmask(to_integer(dec.bitaddr)) <= '1';
                                sfr_wdata <= (others => '1');

        when opcode_callk   =>  sfr_wen(SFR_INDEX_SP)<='1';
                                sfr_wdata <= SP_s + 2;

        when others =>
      end case;
    end if;
  end process;


  -- Main execution unit.

  process(sysclk_s, rst_s)
    variable zero: wordtype := (others => '0');
    variable temp: wordtype;
    variable temp16: unsigned(15 downto 0);
    variable temp7: ioaddrtype;
    variable flagstemp: flags_type;
    variable ipcn: pctype;
    variable opstr: string(1 to 7);

    impure function perform_add(b: in wordtype;c: in std_logic) return wordtype 
    is
      variable a_v, b_v, c_v, add_v: unsigned(8 downto 0);
      variable hadd_v: unsigned(4 downto 0);
      variable oadd_v: unsigned(7 downto 0);
      variable r_v: flags_type;
    begin
      a_v := "0" & A;
      b_v := "0" & b;
      c_v := x"00" & c;
      add_v := a_v + b_v + c_v;
      hadd_v := ("0" &a_v(3 downto 0)) + ("0" & b_v(3 downto 0)) + c_v(4 downto 0);
      oadd_v := ("0" &a_v(6 downto 0)) + ("0" & b_v(6 downto 0)) + c_v(7 downto 0);
      r_v.Z :='0';
      if (add_v(7 downto 0)=x"00") then
        r_v.Z := '1';
      end if;   
      r_v.C  := add_v(8);
      r_v.AC := hadd_v(4);
      r_v.O  := oadd_v(7) xor add_v(8);
      flags   <= r_v;
      return add_v(7 downto 0);
    end function;

    impure function perform_sub( b: in wordtype;
                          c: in std_logic) return wordtype
    is
      variable a_v, b_v, c_v, sub_v: signed(8 downto 0);
      variable ha_v: signed(4 downto 0);
      variable hb_v: signed(4 downto 0);
  
      variable osub_v: signed(7 downto 0);
      variable r_v: flags_type;
    begin
      a_v := '0' & signed(A);
      b_v := '0' & signed(b);
      c_v := x"00" & c;
      sub_v := a_v - b_v - c_v;
  
      r_v.Z :='0';
      if (sub_v(7 downto 0)=x"00") then
        r_v.Z := '1';
      end if;   
        
      r_v.C  := sub_v(8);
      
      ha_v := '0' & signed(a(3 downto 0));
      hb_v := '0' & signed(a(3 downto 0));
      hb_v := hb_v + c_v(4 downto 0);
      if (ha_v < hb_v) then
        r_v.AC := '1';
      else
        r_v.AC := '0';
      end if;
      -- TBD: check if carry is used below.
      osub_v := ("0" &a_v(6 downto 0)) - ("0" & b_v(6 downto 0)) - c_v(7 downto 0);
      r_v.O  := osub_v(7) xor sub_v(8);
      
      flags <= r_v;
      return unsigned(sub_v(7 downto 0));
    end function;

    procedure set_z(data: in unsigned(7 downto 0)) is
    begin
      if data=x"00" then flags.Z <= '1'; else flags.Z <= '0'; end if;
    end procedure;

    procedure assign_and_set_z(data: in unsigned(7 downto 0)) is
      variable temp: unsigned(7 downto 0);
    begin
      temp  := data;
      A <= temp;
      set_z(data);
    end procedure;

    variable t16edge: std_logic;
    variable t16intindex : natural;
    variable unimpl: boolean;

  begin
    
    if rst_s='1' then
      stall_s <= '0';
    elsif rising_edge(sysclk_s) then
    
      cycle <= cycle+1;
      if DEBUG_ENABLED then
        report hstr(std_logic_vector(cycle))&" PC:0x"&hstr(std_logic_vector(IPC)) 
          & " SP:0x" & hstr(std_logic_vector(SP_s))
          & " fl:[" & flagsname(flags) & "] "
          & " A:0x" & hstr(std_logic_vector(A)) 
          & " opcode " & str(opcode_s) & " " & opname(dec);
      end if;

      unimpl := false;
      if stall_s='1' then
        stall_s<='0';
      else
        if int_s='1' and ginten='1' then
          ginten<='0';
          ipcn := ipc;
          writeMem(to_integer(SP_s), ipcn(7 downto 0));
          writeMem( to_integer(SP_s+1) , "000000" & ipcn(9 downto 8));  -- 10-bit PC
        else
        case dec.decoded is
          when opcode_nop     =>
          when opcode_addca   =>  A <= perform_add( x"00", flags.C );
          when opcode_subca   =>  A <= perform_sub( x"00", flags.C );
          when opcode_izsna   =>  A <= perform_add( x"01", '0' );
          when opcode_dzsna   =>  A <= perform_sub( x"01", '0' );
          when opcode_pcadda  =>  null;  -- Jump handled in jump decoder
          when opcode_nota    =>  assign_and_set_z( not A );
          when opcode_nega    =>  assign_and_set_z( neg(A) );
          when opcode_sra     =>  temp := '0' & A(7 downto 1); A <= temp; flags.C <= A(0);
          when opcode_sla     =>  temp := A(6 downto 0) & '0'; A <= temp; flags.C <= A(7);
          when opcode_srca    =>  temp := flags.C & A(7 downto 1); A <= temp; flags.C <= A(0);
          when opcode_slca    =>  temp := A(6 downto 0) & flags.C; A <= temp; flags.C <= A(7);
          when opcode_swapa   =>  A(7 downto 4) <= A(3 downto 0); A(3 downto 0) <= A(7 downto 4);
          when opcode_wdreset =>  null;  -- Watchdog reset not implemented
          when opcode_pushaf  =>  writeMem( to_integer(SP_s) , A);
                                  writeMem( to_integer(SP_s)+1 , "0000" & flags.O & flags.AC & flags.C & flags.Z );

          when opcode_popaf   =>  A         <= readMem( to_integer(SP_s-2) );
                                  temp       := readMem( to_integer(SP_s-1) );
                                  flags.Z   <= temp(0);
                                  flags.C   <= temp(1);
                                  flags.AC   <= temp(2);
                                  flags.O   <= temp(3);

          when opcode_reset   =>  unimpl := true;
          when opcode_stopsys =>  unimpl := true;
          when opcode_stopexe =>  unimpl := true;
          when opcode_engint  =>  ginten <= '1';
          when opcode_disgint =>  ginten <= '0';
          when opcode_ret     =>  null;
          when opcode_reti    =>  ginten <= '1';
          when opcode_mul     =>  unimpl := true;  -- MUL not available on PMS150C
          when opcode_xorioa  =>  null;
          when opcode_movioa  =>  null;
          when opcode_movaio  =>  assign_and_set_z( sfr_read );
          when opcode_retk    =>  A <= dec.immed; 
          when opcode_stt16m  =>  temp16(15 downto 8) := readMem( to_integer(dec.memaddr) );
                                  temp16(7 downto 0) := readMem( to_integer(dec.memaddr) + 1 );
                                  tim16setvalue <= temp16;
                                  tim16set := '1';

          when opcode_ldt16m  =>  unimpl := true;
          when opcode_idxmma  =>  temp := readMem( to_integer( dec.memaddr ) );
                                  writeMem( to_integer(temp), A );
                                  stall_s <= '1';

          when opcode_idxmam  =>  temp := readMem( to_integer( dec.memaddr ) );
                                  A <= readMem( to_integer(temp) );
                                  stall_s <= '1';

          when opcode_addma   =>  temp := readMem(to_integer(dec.memaddr));
                                  temp := temp + A;
                                  writeMem(to_integer(dec.memaddr), temp);
                                  set_z(temp);
          when opcode_subma   =>  temp := readMem(to_integer(dec.memaddr));
                                  temp := temp - A;
                                  writeMem(to_integer(dec.memaddr), temp);
                                  set_z(temp);
          when opcode_addcma  =>  unimpl := true;
          when opcode_subcma  =>  unimpl := true;
          when opcode_andma   =>  temp := readMem(to_integer(dec.memaddr));
                                  temp := temp AND A;
                                  writeMem(to_integer(dec.memaddr),temp);
                                 set_z(temp);


          when opcode_orma    => temp := readMem(to_integer(dec.memaddr));
                                 temp := temp OR A;
                                 writeMem(to_integer(dec.memaddr),temp);
                                 set_z(temp);

          when opcode_xorma   => temp := readMem(to_integer(dec.memaddr));
                                 temp := temp XOR A;
                                 writeMem(to_integer(dec.memaddr),temp);
                                 set_z(temp);
          when opcode_movma   => writeMem( to_integer(dec.memaddr), A);
          when opcode_addam   =>  temp :=  readMem(to_integer(dec.memaddr));
                                  A <= perform_add( temp, '0' );

          when opcode_subam   => temp := readMem(to_integer(dec.memaddr));
                                  A <= perform_sub( temp, '0' );
          when opcode_addcam  => unimpl := true;
          when opcode_subcam  => unimpl := true;
          when opcode_andam   => temp := readMem(to_integer(dec.memaddr));
                                  assign_and_set_z( A and temp );
          when opcode_oram    => temp := readMem(to_integer(dec.memaddr));
                                  assign_and_set_z( A or temp );
          when opcode_xoram   => temp := readMem(to_integer(dec.memaddr));
                                  assign_and_set_z( A xor temp );
          when opcode_movam   => A <= readMem( to_integer(dec.memaddr) );
          when opcode_addcm   => unimpl := true;
          when opcode_subcm   => unimpl := true;
          when opcode_izsnm   => temp := readMem( to_integer(dec.memaddr) );
                                  temp := temp + 1;
                                  writeMem( to_integer(dec.memaddr), temp);
                                  set_z(temp);
          when opcode_dzsnm   => temp := readMem( to_integer(dec.memaddr) );
                                  temp := temp - 1;
                                  writeMem( to_integer(dec.memaddr), temp);
                                  set_z(temp);
          when opcode_incm    =>  temp := readMem( to_integer(dec.memaddr) );
                                  writeMem( to_integer(dec.memaddr), temp + 1);

          when opcode_decm    => temp := readMem( to_integer(dec.memaddr) );
                                  writeMem( to_integer(dec.memaddr), temp - 1);
          when opcode_clearm  => writeMem( to_integer(dec.memaddr), (others => '0'));
          when opcode_xchm    => temp := readMem( to_integer(dec.memaddr) );
                                  writeMem( to_integer(dec.memaddr), A);
                                  A <= temp;
          when opcode_notm    =>  temp := readMem( to_integer(dec.memaddr) );
                                  writeMem( to_integer(dec.memaddr), not temp);
                                  set_z(not temp);
          when opcode_negm    =>  temp := readMem( to_integer(dec.memaddr) );
                                  writeMem( to_integer(dec.memaddr), neg(temp));
                                  set_z(neg(temp));
          when opcode_srm     =>  temp := readMem(to_integer(dec.memaddr));
                                  flags.C <= temp(0);
                                  writeMem( to_integer(dec.memaddr), '0' & temp(7 downto 1));

          when opcode_slm     =>  temp := readMem(to_integer(dec.memaddr));
                                  flags.C <= temp(7);
                                  writeMem( to_integer(dec.memaddr), temp(6 downto 0) & '0');

          when opcode_srcm    =>  temp := readMem(to_integer(dec.memaddr));
                                  flags.C <= temp(0);
                                  writeMem( to_integer(dec.memaddr), flags.C & temp(7 downto 1));

          when opcode_slcm    =>  temp := readMem(to_integer(dec.memaddr));
                                  flags.C <= temp(7);
                                  writeMem( to_integer(dec.memaddr), temp(6 downto 0) & flags.C);

          when opcode_ceqsnam =>  temp := perform_sub(readMem(to_integer(dec.memaddr)),'0');
          when opcode_t0snio  =>  null;
          when opcode_t1snio  =>  null;
          when opcode_set0io  =>  null;
          when opcode_set1io  =>  null;
          when opcode_t0snm   =>  null;
          when opcode_t1snm   =>  null;
          when opcode_set0m   =>  temp := readMem( to_integer(dec.memaddr) );
                                  temp( to_integer( dec.bitaddr ) ) := '0';
                                  writeMem( to_integer(dec.memaddr), temp );
          when opcode_set1m   =>  temp := readMem( to_integer(dec.memaddr) );
                                  temp( to_integer( dec.bitaddr ) ) := '1';
                                  writeMem( to_integer(dec.memaddr), temp );
          when opcode_addak   =>  A <= perform_add(dec.immed, '0');
          when opcode_subak   =>  A <= perform_sub(dec.immed, '0');
          when opcode_ceqsnak =>  temp := perform_sub( dec.immed, '0');
          when opcode_andak   =>  assign_and_set_z( A and dec.immed );
          when opcode_orak    =>  assign_and_set_z( A or dec.immed );
          when opcode_xorak   =>  assign_and_set_z( A xor dec.immed );
          when opcode_movak   =>  A <= dec.immed;
          when opcode_gotok   =>  null;
          when opcode_callk   =>  ipcn := ipc + 1;
                                  writeMem(to_integer(SP_s), ipcn(7 downto 0));
                                  writeMem( to_integer(SP_s+1) , "000000" & ipcn(9 downto 8));  -- 10-bit PC
          
          when others         =>  unimpl := true;
        end case;
        end if;
      
        if unimpl then
          report "UNIMPLEMENTED" severity failure;
        end if;
      end if; -- stall


      -- TBD: this should be moved to its own module once we get
      -- dualport registers.

      -- Generate T16 interrupt
      pt16cnt <= t16cnt;
      t16edge := INTEGS_s(4);
      intreq_s(2) <= '0';
      t16intindex := to_integer (T16M_s(2 downto 0) ) + 8;
      if (pt16cnt(t16intindex)=t16edge and t16cnt(t16intindex)=not t16edge) then
        intreq_s(2) <= '1';
      end if;
      
    end if;

  end process;


  -- TBD: this should be moved to its own module once we get
  -- dualport registers.
  tim16b: block
    signal pres: unsigned(5 downto 0);
    signal en: std_logic;
  begin

    with T16M_s(4 downto 3) select en <=
      '1' when "00",
      pres(1) and not pres(0) when "01",
      pres(3) and not pres(2) and not pres(1) and not pres(0) when "10",
      pres(5) and not pres(4) and not pres(3) and not pres(2) and not pres(1) and not pres(0)when "11",
      '0' when others;


    timer16clk <= sysclk_s;
    
    timer16: process(timer16clk, rst_s)
    begin
      if rst_s='1' then
        pres <= (others => '0');
        t16cnt <=(others => '0');
      elsif rising_edge(timer16clk) then
        if tim16set='1' then
          tim16set:='0';
          t16cnt <= tim16setvalue;
        end if;
        
        
        pres <= pres + 1;
        if en='1' then
          t16cnt <= t16cnt + 1;
        end if;

      end if;
    end process;
    
    
  end block;

  -- PA Inputs directly from pads
  PA_i_s <= PA_io;

  -- PA outputs (TM2 can output on PA3)
  PA_io(0) <= PA_o_s(0);
  PA_io(1) <= PA_o_s(1);
  PA_io(2) <= PA_o_s(2);
  PA_io(3) <= tim2out_s when tm2c_s(3 downto 2)="10" else PA_o_s(3);
  PA_io(4) <= PA_o_s(4);
  PA_io(5) <= PA_o_s(5);
  PA_io(6) <= PA_o_s(6) when not EOSC_CONNECTED else 'X';
  PA_io(7) <= PA_o_s(7) when not EOSC_CONNECTED else 'X';

end sim;
