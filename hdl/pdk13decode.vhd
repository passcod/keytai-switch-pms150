--
--  FPPA PDK13 Microcontroller simulation model - Decoder block
--
--  Based on PDK14 decoder by Alvaro Lopes <alvieboy@alvie.com>
--  Adapted for PDK13 (PMS150C) instruction set (SYM_84B)
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
library work;
use work.txt_util.all;
use work.pdk13pkg.all;

entity pdk13decode is 
  port (
    opcode_i  : in opcodetype;
    valid_i   : in std_logic;
    decoded_o : out opdec_type
  );
end entity;
    
architecture sim of pdk13decode is

  signal dec: opdec_type;

begin

  -- PDK13 instruction encoding (13-bit opcodes)
  -- Bit positions are shifted down by 1 compared to PDK14
  process(opcode_i, valid_i)
    variable dt: std_logic_vector(1 downto 0);
  begin

    dec.immed   <= unsigned(opcode_i(7 downto 0));
    dec.ioaddr  <= unsigned(opcode_i(4 downto 0));
    dec.memaddr <= unsigned(opcode_i(5 downto 0));
    dec.jmpaddr <= unsigned(opcode_i(9 downto 0));
    dec.bitaddr <= unsigned(opcode_i(7 downto 5));

    if valid_i='1' then
      -- NOP: 0x0000
      if opcode_i = "0000000000000" then
        dec.decoded <= opcode_nop;
      -- LDSPTL: 0x0006
      elsif opcode_i = "0000000000110" then
        dec.decoded <= opcode_ldsptl;
      -- LDSPTH: 0x0007
      elsif opcode_i = "0000000000111" then
        dec.decoded <= opcode_ldspth;
      -- Misc instructions 0x001x
      elsif opcode_i(12 downto 4) = "000000001" then
        case opcode_i(3 downto 0) is
          when "0000" => dec.decoded <= opcode_addca;   -- 0x0010
          when "0001" => dec.decoded <= opcode_subca;   -- 0x0011
          when "0010" => dec.decoded <= opcode_izsna;   -- 0x0012
          when "0011" => dec.decoded <= opcode_dzsna;   -- 0x0013
          when "0111" => dec.decoded <= opcode_pcadda;  -- 0x0017
          when "1000" => dec.decoded <= opcode_nota;    -- 0x0018
          when "1001" => dec.decoded <= opcode_nega;    -- 0x0019
          when "1010" => dec.decoded <= opcode_sra;     -- 0x001A
          when "1011" => dec.decoded <= opcode_sla;     -- 0x001B
          when "1100" => dec.decoded <= opcode_srca;    -- 0x001C
          when "1101" => dec.decoded <= opcode_slca;    -- 0x001D
          when "1110" => dec.decoded <= opcode_swapa;   -- 0x001E
          when others => dec.decoded <= opcode_unknown;
        end case;
      -- Misc instructions 0x003x
      elsif opcode_i(12 downto 4) = "000000011" then
        case opcode_i(3 downto 0) is
          when "0000" => dec.decoded <= opcode_wdreset; -- 0x0030
          when "0010" => dec.decoded <= opcode_pushaf;  -- 0x0032
          when "0011" => dec.decoded <= opcode_popaf;   -- 0x0033
          when "0101" => dec.decoded <= opcode_reset;   -- 0x0035
          when "0110" => dec.decoded <= opcode_stopsys; -- 0x0036
          when "0111" => dec.decoded <= opcode_stopexe; -- 0x0037
          when "1000" => dec.decoded <= opcode_engint;  -- 0x0038
          when "1001" => dec.decoded <= opcode_disgint; -- 0x0039
          when "1010" => dec.decoded <= opcode_ret;     -- 0x003A
          when "1011" => dec.decoded <= opcode_reti;    -- 0x003B
          when "1100" => dec.decoded <= opcode_mul;     -- 0x003C (not available on PMS150C)
          when others => dec.decoded <= opcode_unknown;
        end case;
      -- IO operations with A: 0x006.-0x00B.
      -- XOR IO, A: 0x006.-0x007. (0 0 0 0 0 0 1 1 IO[4:0])
      elsif opcode_i(12 downto 6) = "0000001" and opcode_i(5) = '1' then
        dec.decoded <= opcode_xorioa;
      -- MOV IO, A: 0x008.-0x009. (0 0 0 0 0 1 0 0 IO[4:0])
      elsif opcode_i(12 downto 6) = "0000010" and opcode_i(5) = '0' then
        dec.decoded <= opcode_movioa;
      -- MOV A, IO: 0x00A.-0x00B. (0 0 0 0 0 1 0 1 IO[4:0])
      elsif opcode_i(12 downto 6) = "0000010" and opcode_i(5) = '1' then
        dec.decoded <= opcode_movaio;
      -- 16-bit memory operations: 0x00C.-0x00F.
      elsif opcode_i(12 downto 7) = "000001" then
        dt := opcode_i(6) & opcode_i(0);
        dec.memaddr <= unsigned(opcode_i(4 downto 0) & '0'); -- Word aligned, 5-bit word address
        case dt is
          when "00"   => dec.decoded <= opcode_stt16m;  -- STT16 M
          when "01"   => dec.decoded <= opcode_ldt16m;  -- LDT16 M
          when "10"   => dec.decoded <= opcode_idxmma;  -- IDXM M, A
          when "11"   => dec.decoded <= opcode_idxmam;  -- IDXM A, M
          when others => dec.decoded <= opcode_unknown;
        end case;
      -- RET k: 0x01.. (0 0 0 0 1 k[7:0])
      elsif opcode_i(12 downto 8) = "00001" then
        dec.decoded <= opcode_retk;
      -- Bit operations with memory: 0x02..-0x03..
      -- T0SN M.n: 0x02.. bit 4=0 (0 0 0 1 0 n[2:0] 0 M[3:0])
      elsif opcode_i(12 downto 8) = "00010" and opcode_i(4) = '0' then
        dec.decoded <= opcode_t0snm;
        dec.memaddr <= unsigned("00" & opcode_i(3 downto 0));
        dec.bitaddr <= unsigned(opcode_i(7 downto 5));
      -- T1SN M.n: 0x02.. bit 4=1 (0 0 0 1 0 n[2:0] 1 M[3:0])
      elsif opcode_i(12 downto 8) = "00010" and opcode_i(4) = '1' then
        dec.decoded <= opcode_t1snm;
        dec.memaddr <= unsigned("00" & opcode_i(3 downto 0));
        dec.bitaddr <= unsigned(opcode_i(7 downto 5));
      -- SET0 M.n: 0x03.. bit 4=0 (0 0 0 1 1 n[2:0] 0 M[3:0])
      elsif opcode_i(12 downto 8) = "00011" and opcode_i(4) = '0' then
        dec.decoded <= opcode_set0m;
        dec.memaddr <= unsigned("00" & opcode_i(3 downto 0));
        dec.bitaddr <= unsigned(opcode_i(7 downto 5));
      -- SET1 M.n: 0x03.. bit 4=1 (0 0 0 1 1 n[2:0] 1 M[3:0])
      elsif opcode_i(12 downto 8) = "00011" and opcode_i(4) = '1' then
        dec.decoded <= opcode_set1m;
        dec.memaddr <= unsigned("00" & opcode_i(3 downto 0));
        dec.bitaddr <= unsigned(opcode_i(7 downto 5));
      -- Operations with A and memory: 0x04..-0x07..
      elsif opcode_i(12 downto 9) = "0010" then
        case opcode_i(8 downto 6) is
          when "000"  => dec.decoded <= opcode_addma;   -- ADD M, A
          when "001"  => dec.decoded <= opcode_subma;   -- SUB M, A
          when "010"  => dec.decoded <= opcode_addcma;  -- ADDC M, A
          when "011"  => dec.decoded <= opcode_subcma;  -- SUBC M, A
          when "100"  => dec.decoded <= opcode_andma;   -- AND M, A
          when "101"  => dec.decoded <= opcode_orma;    -- OR M, A
          when "110"  => dec.decoded <= opcode_xorma;   -- XOR M, A
          when "111"  => dec.decoded <= opcode_movma;   -- MOV M, A
          when others => dec.decoded <= opcode_unknown;
        end case;
      elsif opcode_i(12 downto 9) = "0011" then
        case opcode_i(8 downto 6) is
          when "000"  => dec.decoded <= opcode_addam;   -- ADD A, M
          when "001"  => dec.decoded <= opcode_subam;   -- SUB A, M
          when "010"  => dec.decoded <= opcode_addcam;  -- ADDC A, M
          when "011"  => dec.decoded <= opcode_subcam;  -- SUBC A, M
          when "100"  => dec.decoded <= opcode_andam;   -- AND A, M
          when "101"  => dec.decoded <= opcode_oram;    -- OR A, M
          when "110"  => dec.decoded <= opcode_xoram;   -- XOR A, M
          when "111"  => dec.decoded <= opcode_movam;   -- MOV A, M
          when others => dec.decoded <= opcode_unknown;
        end case;
      -- Operations with memory: 0x08..-0x0B..
      elsif opcode_i(12 downto 9) = "0100" then
        case opcode_i(8 downto 6) is
          when "000"  => dec.decoded <= opcode_addcm;   -- ADDC M
          when "001"  => dec.decoded <= opcode_subcm;   -- SUBC M
          when "010"  => dec.decoded <= opcode_izsnm;   -- IZSN M
          when "011"  => dec.decoded <= opcode_dzsnm;   -- DZSN M
          when "100"  => dec.decoded <= opcode_incm;    -- INC M
          when "101"  => dec.decoded <= opcode_decm;    -- DEC M
          when "110"  => dec.decoded <= opcode_clearm;  -- CLEAR M
          when "111"  => dec.decoded <= opcode_xchm;    -- XCH M
          when others => dec.decoded <= opcode_unknown;
        end case;
      elsif opcode_i(12 downto 9) = "0101" then
        case opcode_i(8 downto 6) is
          when "000"  => dec.decoded <= opcode_notm;    -- NOT M
          when "001"  => dec.decoded <= opcode_negm;    -- NEG M
          when "010"  => dec.decoded <= opcode_srm;     -- SR M
          when "011"  => dec.decoded <= opcode_slm;     -- SL M
          when "100"  => dec.decoded <= opcode_srcm;    -- SRC M
          when "101"  => dec.decoded <= opcode_slcm;    -- SLC M
          when "110"  => dec.decoded <= opcode_ceqsnam; -- CEQSN A, M
          when others => dec.decoded <= opcode_unknown;
        end case;
      -- Bit operations with IO: 0x0C..-0x0F..
      elsif opcode_i(12 downto 10) = "011" then
        case opcode_i(9 downto 8) is
          when "00"   => dec.decoded <= opcode_t0snio;  -- T0SN IO.n
          when "01"   => dec.decoded <= opcode_t1snio;  -- T1SN IO.n
          when "10"   => dec.decoded <= opcode_set0io;  -- SET0 IO.n
          when "11"   => dec.decoded <= opcode_set1io;  -- SET1 IO.n
          when others => dec.decoded <= opcode_unknown;
        end case;
      -- Operations with A and 8-bit literal: 0x10..-0x17..
      elsif opcode_i(12 downto 11) = "10" then
        case opcode_i(10 downto 8) is
          when "000"  => dec.decoded <= opcode_addak;   -- ADD A, k
          when "001"  => dec.decoded <= opcode_subak;   -- SUB A, k
          when "010"  => dec.decoded <= opcode_ceqsnak; -- CEQSN A, k
          when "100"  => dec.decoded <= opcode_andak;   -- AND A, k
          when "101"  => dec.decoded <= opcode_orak;    -- OR A, k
          when "110"  => dec.decoded <= opcode_xorak;   -- XOR A, k
          when "111"  => dec.decoded <= opcode_movak;   -- MOV A, k
          when others => dec.decoded <= opcode_unknown;
        end case;
      -- Control transfers: 0x18..-0x1F..
      elsif opcode_i(12 downto 10) = "110" then
        dec.decoded <= opcode_gotok;  -- GOTO k
      elsif opcode_i(12 downto 10) = "111" then
        dec.decoded <= opcode_callk;  -- CALL k
      else
        dec.decoded <= opcode_unknown;
      end if;
    else
      dec.decoded <= opcode_nop;
    end if;
  end process;

  decoded_o <= dec;

end sim;
