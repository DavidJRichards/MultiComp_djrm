-- This file is copyright by Grant Searle 2014
-- Grant Searle's web site http://searle.hostei.com/grant/    
-- Grant Searle's "multicomp" page at http://searle.hostei.com/grant/Multicomp/index.html
--
-- Changes to this code by Doug Gilliland 2019
--	48K (external) RAM version
--
-- Jumper in pin 85 to ground (adjacent pin) of the FPGA selects the VDU/Serial port
-- Install to make serial port default
-- Remove jumper to make the VDU default
--
-- 115,200 baud serial port
-- Hardware handshake RTS/CTS
--
-- djrm conversion to sdram begin, june23 2020
-- pll and sdram fsm copied from Sytse van Slooten's pdp2011 DE0
--
-- Copyright (c) 2008-2019 Sytse van Slooten
--
-- Permission is hereby granted to any person obtaining a copy of these VHDL source files and
-- other language source files and associated documentation files ("the materials") to use
-- these materials solely for personal, non-commercial purposes.
-- You are also granted permission to make changes to the materials, on the condition that this
-- copyright notice is retained unchanged.
--
-- The materials are distributed in the hope that they will be useful, but WITHOUT ANY WARRANTY;
-- without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
--

library ieee;
use ieee.std_logic_1164.all;
use  IEEE.STD_LOGIC_ARITH.all;
use  IEEE.STD_LOGIC_UNSIGNED.all;

entity Microcomputer is
	port(
		n_reset		: in std_logic;
		i_CLOCK_50	: in std_logic;

		-- SRAM Address/Data bus
--		sramData		: inout std_logic_vector(7 downto 0);
--		sramAddress	: out std_logic_vector(18 downto 0);
--		n_sRamWE		: out std_logic;
--		n_sRamCS		: out std_logic;
--		n_sRamOE		: out std_logic;

		-- Not using the SD RAM but making sure that it's not active
--		n_sdRamCas	: out std_logic := '1';		-- CAS on schematic
--		n_sdRamRas	: out std_logic := '1';		-- RAS
--		n_sdRamWe	: out std_logic := '1';		-- SDWE
--		n_sdRamCe	: out std_logic := '1';		-- SD_NCS0
--		sdRamClk		: out std_logic := '1';		-- SDCLK0
--		sdRamClkEn	: out std_logic := '1';		-- SDCKE0


      dram_addr : out std_logic_vector(11 downto 0);
      dram_dq : inout std_logic_vector(15 downto 0);
      dram_cas_n : out std_logic;
      dram_ras_n : out std_logic;
      dram_we_n : out std_logic;
      dram_cs_n : out std_logic;
      dram_clk : out std_logic;
      dram_cke : out std_logic;
      dram_ba_1 : out std_logic;
      dram_ba_0 : out std_logic;
      dram_udqm : out std_logic;
      dram_ldqm : out std_logic;
 

		
		rxd1			: in std_logic;
		txd1			: out std_logic;
		cts1			: in std_logic;
		rts1			: out std_logic;
		
		videoR0		: out std_logic;
		videoG0		: out std_logic;
		videoB0		: out std_logic;
		videoR1		: out std_logic;
		videoG1		: out std_logic;
		videoB1		: out std_logic;
		hSync			: out std_logic;
		vSync			: out std_logic;

		ps2Clk		: inout std_logic;
		ps2Data		: inout std_logic;

		i_pbutton   : in std_logic_vector(1 downto 0);
		o_BUZZER        : out std_logic;
		serSelect	: in std_logic := '1'
	);
end Microcomputer;

architecture struct of Microcomputer is

	signal n_WR							: std_logic;
	signal n_RD							: std_logic;
	signal cpuAddress					: std_logic_vector(15 downto 0);
	signal cpuDataOut					: std_logic_vector(7 downto 0);
	signal cpuDataIn					: std_logic_vector(7 downto 0);

	signal basRomData					: std_logic_vector(7 downto 0);
	signal interface1DataOut		: std_logic_vector(7 downto 0);
	signal internalRam1DataOut		: std_logic_vector(7 downto 0);
	signal interface2DataOut		: std_logic_vector(7 downto 0);
	
	signal w_displayed_number		: std_logic_vector(31 downto 0);

	signal n_memWR						: std_logic :='1';
	signal n_memRD 					: std_logic :='1';


signal c0 : std_logic;
signal dram_match : std_logic;
signal dram_counter : integer range 0 to 32767;
signal dram_wait : integer range 0 to 15;
type dram_fsm_type is (
   dram_init,
   dram_poweron,
   dram_pwron_pre, dram_pwron_prew,
   dram_pwron_ref, dram_pwron_refw,
   dram_pwron_mrs, dram_pwron_mrsw,
   dram_c1,
   dram_c2,
   dram_c3,
   dram_c4,
   dram_c5,
   dram_c6,
   dram_c7,
   dram_c8,
   dram_c9,
   dram_c10,
   dram_c11,
   dram_c12,
   dram_c13,
   dram_c14,
   dram_c15,
   dram_c16,
   dram_idle
);
signal dram_fsm : dram_fsm_type := dram_init;


	signal n_int1						: std_logic :='1';	
	signal n_int2						: std_logic :='1';	
	
	signal n_internalRamCS			: std_logic :='1';
	signal n_externalRamCS			: std_logic :='1';
	signal n_basRomCS					: std_logic :='1';
	signal n_interface1CS			: std_logic :='1';
	signal n_interface2CS			: std_logic :='1';
	
	signal cpuCount					: std_logic_vector(5 downto 0); 
	signal cpuClock					: std_logic;
	signal resetLow					: std_logic := '1';

    signal serialCount         : std_logic_vector(15 downto 0) := x"0000";
    signal serialCount_d       : std_logic_vector(15 downto 0);
    signal serialEn            : std_logic;

	signal cpuclk : std_logic := '0';
	signal cpureset : std_logic := '1';
	signal cpuresetlength : integer range 0 to 63 := 63;
	signal slowreset : std_logic;
	signal slowresetdelay : integer range 0 to 4095 := 4095;
	signal vtreset : std_logic := '1';
signal ifetch: std_logic;
signal iwait: std_logic;
--signal reset: std_logic; djrm now n-reset
signal addr : std_logic_vector(21 downto 0);
signal addrq : std_logic_vector(21 downto 0);
signal dati : std_logic_vector(15 downto 0);
signal dato : std_logic_vector(15 downto 0);
signal control_dati : std_logic;
signal control_dato : std_logic;
signal control_datob : std_logic;


component pll is
   port(
      inclk0 : in std_logic := '0';
      c0 : out std_logic
   );
end component;
	 
begin

-- setup pll
   pll0: pll port map(
      inclk0 => i_CLOCK_50,
      c0 => c0
   );

	-- Debounce the reset line
--	DebounceResetSwitch	: entity work.Debouncer
--	port map (
--		i_CLOCK_50	=> i_CLOCK_50,
--		i_PinIn		=> n_reset,
--		o_PinOut		=> resetLow
--	);

	
	-- SRAM
--	sramAddress(18 downto 16) <= "010"; -- A17 is CE2 on AS6C1008 , needs +ve enable
--	sramAddress(15 downto 0) <= cpuAddress(15 downto 0);
--	sramAddress(18 downto 16) <= "010"; -- A17 is CE2 on AS6C1008 , needs +ve enable
--	sramData <= cpuDataOut when n_WR='0' else (others => 'Z');
--	n_sRamWE	<= (n_memWR or n_externalRamCS);
--	n_sRamOE <= (n_memRD or n_externalRamCS);
--	n_sRamCS <= n_externalRamCS;
	

	-- SDRAM
	addr(15 downto 0) <= cpuAddress(15 downto 0);
	addr(21 downto 16) <= "000000";

	dato(7 downto 0) <= cpuDataOut when n_WR='0' else (others => 'Z');
	dato(15 downto 8) <= cpuDataOut when n_WR='0' else (others => 'Z');
	
	-- ____________________________________________________________________________________
	-- 6809 CPU
	-- works with Version 1.26
	-- Does not work with Version 1.28 FPGA core
	cpu1 : entity work.cpu09
		port map(
			clk => not(cpuClock),
--			rst => not resetLow,
			rst => cpureset, -- djrm, from sram fsm initialisation
			rw => n_WR,
			addr => cpuAddress,
			data_in => cpuDataIn,
			data_out => cpuDataOut,
			halt => '0',
			hold => '0',
			irq => '0',
			firq => '0',
			nmi => '0'
		); 
	
	-- ____________________________________________________________________________________
	-- BASIC ROM	
--	rom1 : entity work.M6809_EXT_BASIC_ROM -- 8KB BASIC
--	rom1 : entity work.M6809_CAMELFORTH_ROM -- Camelforth @ 0xE000
  rom1 : entity work.M6809_SIMON_ROM -- Buggy @ 0xE000
		port map(
			address => cpuAddress(12 downto 0),
			clock => i_CLOCK_50,
			q => basRomData
		);
	
	-- ____________________________________________________________________________________
	-- RAM GOES HERE
 	ram1: entity work.InternalRam4K
		port map
		(
			address 	=> cpuAddress(11 downto 0),
			clock 	=> i_CLOCK_50,
			data 		=> cpuDataOut,
			wren 		=> not(n_memWR or n_internalRamCS),
			q 			=> internalRam1DataOut
		);
			
	-- ____________________________________________________________________________________
	-- INPUT/OUTPUT DEVICES
	-- Grant's VGA driver
	-- Removed the Composite video output
	io1 : entity work.SBCTextDisplayRGB
		port map (
--			n_reset => resetLow,
			n_reset => not vtreset, -- djrm from dram fsm
			clk => i_CLOCK_50,
			
			-- RGB CompVideo signals
			hSync => hSync,
			vSync => vSync,
			videoR0 => videoR0,
			videoR1 => videoR1,
			videoG0 => videoG0,
			videoG1 => videoG1,
			videoB0 => videoB0,
			videoB1 => videoB1,
			
			n_wr => n_interface1CS or cpuClock or n_WR,
			n_rd => n_interface1CS or cpuClock or (not n_WR),
			n_int => n_int1,
			regSel => cpuAddress(0),
			dataIn => cpuDataOut,
			dataOut => interface1DataOut,
			ps2clk => ps2Clk,
			ps2Data => ps2Data
		);
	
	-- Replaced Grant's bufferedUART with Neal Crook's version which uses clock enables instead of clock
	io2 : entity work.bufferedUART
		port map(
			clk => i_CLOCK_50,
			n_wr => n_interface2CS or cpuClock or n_WR,
			n_rd => n_interface2CS or cpuClock or (not n_WR),
			n_int => n_int2,
			regSel => cpuAddress(0),
			dataIn => cpuDataOut,
			dataOut => interface2DataOut,
			rxClkEn  => serialEn,
			txClkEn => serialEn,			
			rxd => rxd1,
			txd => txd1,
			n_cts => cts1,
			n_rts => rts1
		);
	
	-- ____________________________________________________________________________________
	-- MEMORY READ/WRITE LOGIC
	n_memRD <= not(cpuClock) nand n_WR;
	n_memWR <= not(cpuClock) nand (not n_WR);
	
	control_dato	<= not n_WR; 
	control_dati 	<= n_WR;
	
	-- ____________________________________________________________________________________
	-- CHIP SELECTS
	-- Jumper Pin_85 selects whether UART or VDU are default
	n_basRomCS <= '0' when cpuAddress(15 downto 13) = "111" else '1'; --8K at top of memory
	n_interface1CS <= '0' when ((cpuAddress(15 downto 1) = "111111111101000" and serSelect = '1') or 
										 (cpuAddress(15 downto 1) = "111111111101001" and serSelect = '0')) else '1'; -- 2 bytes FFD0-FFD1
	n_interface2CS <= '0' when ((cpuAddress(15 downto 1) = "111111111101001" and serSelect = '1') or 
										 (cpuAddress(15 downto 1) = "111111111101000" and serSelect = '0')) else '1'; -- 2 bytes FFD2-FFD3
	n_internalRamCS <= '0' when cpuAddress(15 downto 12) = "0000" else '1'; -- 4K at bottom of memory (0x0 to 0xfff)
--	dram_match      <= '0' when cpuAddress(15 downto 12) = "0001" else '1'; -- next 4K at bottom of memory (0x1000 to 0x1fff)
	dram_match      <= '1' when cpuAddress(15 downto 12) = "0001" else '0'; -- next 4K at bottom of memory (0x1000 to 0x1fff)
	ifetch <= dram_match;
--	n_externalRamCS <= '0' when cpuAddress(15 downto 12) = "0001" else '1'; -- next 4K at bottom of memory (0x1000 to 0x1fff)

	--	n_internalRamCS <= not n_basRomCS;
--	n_externalRamCS <= not n_basRomCS;
	
	-- ____________________________________________________________________________________
	-- BUS ISOLATION
	-- Order matters since SRAM overlaps I/O chip selects
	cpuDataIn <=
		interface1DataOut 		when n_interface1CS = '0' 		else
		interface2DataOut 		when n_interface2CS = '0' 		else
		basRomData 					when n_basRomCS = '0'			else
		internalRam1DataOut 		when n_internalRamCS = '0' 	else
--		sramData 					when n_externalRamCS = '0' 	else
		dati(7 downto 0)        when dram_match = '1'         else
		x"FF";
	
	-- ____________________________________________________________________________________
	-- SYSTEM CLOCKS
--process (i_CLOCK_50)
--	begin
--		if rising_edge(i_CLOCK_50) then
--			if cpuCount < 40 then -- 4 = 10MHz, 3 = 12.5MHz, 2=16.6MHz, 1=25MHz
--				cpuCount <= cpuCount + 1;
--			else
--				cpuCount <= (others=>'0');
--			end if;
--			if cpuCount < 20 then -- 2 when 10MHz, 2 when 12.5MHz, 2 when 16.6MHz, 1 when 25MHz
--				cpuClock <= '0';
--			else
--				cpuClock <= '1';
--			end if;
--		end if;
--	end process;
	
	
	-- Baud Rate CLOCK SIGNALS
	
baud_div: process (serialCount_d, serialCount)
    begin
        serialCount_d <= serialCount + 2416;
    end process;

process (i_CLOCK_50)
	begin
		if rising_edge(i_CLOCK_50) then
        -- Enable for baud rate generator
        serialCount <= serialCount_d;
        if serialCount(15) = '0' and serialCount_d(15) = '1' then
            serialEn <= '1';
        else
            serialEn <= '0';
        end if;
		end if;
	end process;


process(c0)
   begin
      if c0='1' and c0'event then
         if slowreset = '1' then
            dram_fsm <= dram_init;
            dram_cs_n <= '0';
            dram_ras_n <= '1';
            dram_cas_n <= '1';
            dram_we_n <= '1';
            dram_addr <= (others => '0');

            dram_udqm <= '1';
            dram_ldqm <= '1';
            dram_ba_1 <= '0';
            dram_ba_0 <= '0';

            cpuclk <= '0';
				cpuClock <= '0'; -- djrm
            cpureset <= '1';
            cpuresetlength <= 63;

            if i_pbutton(1) = '0' then
               o_BUZZER <= '0';
            else
               o_BUZZER <= '1';
            end if;

--            if sw(0) = '0' then
--               have_rl <= 1;
--               have_rk <= 0;
--            else
--               have_rl <= 0;
--               have_rk <= 1;
--            end if;
         else

            case dram_fsm is

               when dram_init =>
                  dram_cs_n <= '0';
                  dram_ras_n <= '1';
                  dram_cas_n <= '1';
                  dram_we_n <= '1';
                  dram_addr <= (others => '0');

                  dram_udqm <= '1';
                  dram_ldqm <= '1';
                  dram_ba_1 <= '0';
                  dram_ba_0 <= '0';

                  cpureset <= '1';
                  cpuresetlength <= 8;
                  dram_counter <= 32767;
                  dram_fsm <= dram_poweron;

               when dram_poweron =>
                  dram_cs_n <= '0';
                  dram_ras_n <= '1';
                  dram_cas_n <= '1';
                  dram_we_n <= '1';
                  dram_addr <= (others => '0');

                  dram_udqm <= '1';
                  dram_ldqm <= '1';
                  dram_ba_1 <= '0';
                  dram_ba_0 <= '0';

                  if dram_counter = 0 then
                     dram_fsm <= dram_pwron_pre;
                  else
                     dram_counter <= dram_counter - 1;
                  end if;

               when dram_pwron_pre =>
                  dram_cs_n <= '0';
                  dram_ras_n <= '0';
                  dram_cas_n <= '1';
                  dram_we_n <= '0';
                  dram_addr(10) <= '1';

                  dram_udqm <= '1';
                  dram_ldqm <= '1';
                  dram_ba_1 <= '0';
                  dram_ba_0 <= '0';
--                  dram_addr(12) <= '0';
                  dram_addr(11) <= '0';
                  dram_addr(9 downto 0) <= (others => '0');

                  dram_wait <= 4;
                  dram_fsm <= dram_pwron_prew;

               when dram_pwron_prew =>
                  dram_cs_n <= '1';
                  if dram_wait = 0 then
                     dram_fsm <= dram_pwron_ref;
                     dram_counter <= 20;
                  else
                     dram_wait <= dram_wait - 1;
                  end if;

               when dram_pwron_ref =>
                  dram_cs_n <= '0';
                  dram_ras_n <= '0';
                  dram_cas_n <= '0';
                  dram_we_n <= '1';
                  dram_addr <= (others => '0');

                  dram_udqm <= '1';
                  dram_ldqm <= '1';
                  dram_ba_1 <= '0';
                  dram_ba_0 <= '0';

                  dram_wait <= 15;
                  dram_fsm <= dram_pwron_refw;

               when dram_pwron_refw =>
                  dram_cs_n <= '1';
                  if dram_wait = 0 then
                     if dram_counter = 0 then
                        dram_fsm <= dram_pwron_mrs;
                     else
                        dram_counter <= dram_counter - 1;
                        dram_fsm <= dram_pwron_ref;
                     end if;
                  else
                     dram_wait <= dram_wait - 1;
                  end if;

               when dram_pwron_mrs =>
                  dram_cs_n <= '0';
                  dram_ras_n <= '0';
                  dram_cas_n <= '0';
                  dram_we_n <= '0';

                  dram_addr(11 downto 7) <= (others => '0');
                  dram_addr(6 downto 4) <= "011";          -- cas length 3
                  dram_addr(3) <= '0';                     -- sequential
                  dram_addr(2 downto 0) <= "000";          -- length 0

                  dram_udqm <= '1';
                  dram_ldqm <= '1';
                  dram_ba_1 <= '0';
                  dram_ba_0 <= '0';

                  dram_wait <= 4;
                  dram_fsm <= dram_pwron_mrsw;

               when dram_pwron_mrsw =>
                  dram_cs_n <= '1';
                  if dram_wait = 0 then
                     dram_fsm <= dram_idle;
                  else
                     dram_wait <= dram_wait - 1;
                  end if;

               when dram_idle =>
                  dram_cs_n <= '1';
                  dram_ras_n <= '1';
                  dram_cas_n <= '1';
                  dram_we_n <= '1';
                  dram_addr(10) <= '0';

                  dram_udqm <= '1';
                  dram_ldqm <= '1';
                  dram_ba_1 <= '0';
                  dram_ba_0 <= '0';
--                  dram_addr(12) <= '0';
                  dram_addr(11) <= '0';
                  dram_addr(9 downto 0) <= (others => '0');

                  dram_fsm <= dram_c1;

               when dram_c1 =>

						cpuclk <= '1';
						cpuClock <= '1'; -- djrm

                  if cpuresetlength = 0 then
                     cpureset <= '0';
                  else
                     cpuresetlength <= cpuresetlength - 1;
                  end if;
                  dram_fsm <= dram_c2;

               when dram_c2 =>
                  dram_dq <= (others => 'Z');
                  dram_fsm <= dram_c3;

               when dram_c3 =>
                  dram_fsm <= dram_c4;

               when dram_c4 =>
                  dram_fsm <= dram_c5;

               when dram_c5 =>
                  dram_fsm <= dram_c6;

               when dram_c6 =>
                  -- read, t1-t2
                  if ifetch = '1' then
                     addrq <= addr;
                  end if;

                  if dram_match = '1' and control_dati = '1' then
                     -- activate command
--    o_BUZZER <= '0'; -- read
                     dram_cs_n <= '0';
                     dram_ras_n <= '0';
                     dram_cas_n <= '1';
                     dram_we_n <= '1';
--                     dram_addr(12) <= '0';
                     dram_addr(11 downto 0) <= addr(20 downto 9);

                     dram_udqm <= '0';
                     dram_ldqm <= '0';
                     dram_ba_1 <= '0';
                     dram_ba_0 <= addr(21);
                  end if;

                  -- write, t1-t2
                  if dram_match = '1' and control_dato = '1' then
                     -- activate command
--    o_BUZZER <= '0'; --write
                     dram_cs_n <= '0';
                     dram_ras_n <= '0';
                     dram_cas_n <= '1';
                     dram_we_n <= '1';
--                     dram_addr(12) <= '0';
                     dram_addr(11 downto 0) <= addr(20 downto 9);

                     dram_udqm <= '0';
                     dram_ldqm <= '0';
                     dram_ba_1 <= '0';
                     dram_ba_0 <= addr(21);
                  end if;

                  if dram_match = '0' or (control_dato = '0' and control_dati = '0') then
                     -- auto refresh command
                     dram_cs_n <= '0';
                     dram_ras_n <= '0';
                     dram_cas_n <= '0';
                     dram_we_n <= '1';
                  end if;

                  dram_fsm <= dram_c7;

               when dram_c7 =>
                  -- t2-t3 - set nop command
                  dram_cs_n <= '1';
                  dram_ras_n <= '1';
                  dram_cas_n <= '1';
                  dram_we_n <= '1';

                  dram_fsm <= dram_c8;

               when dram_c8 =>

                  -- read, t3-t4
                  if dram_match = '1' and control_dati = '1' then
                     -- reada command
--     o_BUZZER <= '0'; --read
                    dram_cs_n <= '0';
                     dram_ras_n <= '1';
                     dram_cas_n <= '0';
                     dram_we_n <= '1';
--                     dram_addr(12) <= '0';
                     dram_addr(11) <= '0';
                     dram_addr(10) <= '1';
                     dram_addr(9) <= '1';
                     dram_addr(8) <= '0';
                     dram_addr(7 downto 0) <= addr(8 downto 1);

                     dram_udqm <= '0';
                     dram_ldqm <= '0';
                     dram_ba_1 <= '0';
                     dram_ba_0 <= addr(21);
                  end if;

                  -- write, t3-t4
                  if dram_match = '1' and control_dato = '1' then
                     -- writea command
--    o_BUZZER <= '1'; --write
                     dram_cs_n <= '0';
                     dram_ras_n <= '1';
                     dram_cas_n <= '0';
                     dram_we_n <= '0';
--                     dram_addr(12) <= '0';
                     dram_addr(11) <= '0';
                     dram_addr(10) <= '1';
                     dram_addr(9) <= '1';
                     dram_addr(8) <= '0';
                     dram_addr(7 downto 0) <= addr(8 downto 1);
                     dram_udqm <= '0';
                     dram_ldqm <= '0';
                     if control_datob = '1' then
                        if addr(0) = '0' then
                           dram_udqm <= '1';
                        else
                           dram_ldqm <= '1';
                        end if;
                     end if;
                     dram_ba_1 <= '0';
                     dram_ba_0 <= addr(21);
                     dram_dq <= dato;
                  end if;

                  dram_fsm <= dram_c9;

               cpuclk <= '0';
               cpuClock <= '0';

               when dram_c9 =>

                  -- read/write, t4-t5 - set nop command and deselect
                  dram_cs_n <= '1';
                  dram_ras_n <= '1';
                  dram_cas_n <= '1';
                  dram_we_n <= '1';

                  dram_fsm <= dram_c10;

               when dram_c10 =>
                  dram_fsm <= dram_c11;

               when dram_c11 =>
                  dram_fsm <= dram_c12;

               when dram_c12 =>
                  dram_fsm <= dram_c13;

               when dram_c13 =>
                  -- read, t5-t6
                  if dram_match = '1' and control_dati = '1' then
--                     dati <= x"1234"; --dram_dq;
                     dati <= dram_dq;
                  end if;
                  dram_fsm <= dram_c14;

               when dram_c14 =>
                  dram_fsm <= dram_c1;

               when others =>
                  null;

            end case;

         end if;
      end if;
   end process;

   process (c0)
   begin
      if c0='1' and c0'event then
         if n_reset = '0' then -- djrm changed polarity
            slowreset <= '1';
            slowresetdelay <= 4095;
         else
            if slowresetdelay = 0 then
               slowreset <= '0';
               vtreset <= '0';
            else
               slowreset <= '1';
               slowresetdelay <= slowresetdelay - 1;
            end if;
         end if;
      end if;
   end process;

end;
