library ieee;
use ieee.numeric_std.all;
use ieee.std_logic_1164.all;

library vjtag;
library pll_core;

--============================================================================
--  Virtual JTAG Port Generation Block
--============================================================================
-- Generate a vuirt JTAG block to allow control and data to flow from
-- the FPGA under development.
-- Version: 0.0.0 Initial Commit - half dead time block - compiles -Shaun
------------------------------------------------------------------------------

entity FRED is
Port (

	clk_24	: in std_logic;
	RST_IN	: in std_logic;
	
	UH_OUT		: out	std_logic;
	UL_OUT		: out std_logic;
	VH_OUT		: out std_logic;
	VL_OUT		: out std_logic;
	WH_OUT		: out std_logic;
	WL_OUT		: out	std_logic;

	A_OUT			: out	std_logic_vector(1 downto 0);
	RES_OUT 		: out	std_logic_vector(1 downto 0);
	SAMPLE_OUT	: out std_logic;
	WR_OUT		: out std_logic;
	SCLK_OUT 	: out std_logic;
	MISO_IN		: in std_logic;
	MOSI_OUT		: out std_logic;
	
	leds_out		: out std_logic_vector(7 downto 0) := X"00"
	
	--DEBUG_ENA	: in std_logic
	
	);
end entity FRED;

architecture rtl of FRED is

	-- clocks    
	signal clk_40, clk_2k : std_logic;

	signal tdi_sig, tdo_sig, tck, sdr_valid, data_ready, load_dr	:	std_logic;
	signal ir	: std_logic_vector(7 downto 0);
	signal dr	: std_logic_vector(31 downto 0) := X"87654321";
	
	-- sine lookup signals
	signal position 	: std_logic_vector(9 downto 0);

	-- pwm signals
	signal	pwm_uh	: std_logic_vector(9 downto 0);
	signal	pwm_ul	: std_logic_vector(9 downto 0);
	signal	pwm_vh	: std_logic_vector(9 downto 0);
	signal	pwm_vl	: std_logic_vector(9 downto 0);
	signal	pwm_wh	: std_logic_vector(9 downto 0);
	signal	pwm_wl	: std_logic_vector(9 downto 0);

	signal	pwm_masked_ul, pwm_masked_uh	: std_logic_vector(9 downto 0);
	signal	pwm_masked_vl, pwm_masked_vh	: std_logic_vector(9 downto 0);
	signal	pwm_masked_wl, pwm_masked_wh	: std_logic_vector(9 downto 0);

	signal	uh_signal, ul_signal	: std_logic := '0';
	signal	vh_signal, vl_signal	: std_logic := '0';
	signal	wh_signal, wl_signal	: std_logic := '0';
	
	-- GPIO signals
	signal gpio_out, gpio_in		: std_logic_vector(7 downto 0);	

	-- Resolver Signals
	signal resolver_signal	: std_logic_vector(23 downto 0);
	signal res					: std_logic_vector(1 downto 0) := (others => '0');
	signal address				: std_logic_vector(1 downto 0) := (others => '0');
	signal resolver_control	: std_logic_vector(3 downto 0);
	signal resolver_position : std_logic_vector(15 downto 0);
	signal sample_signal		: std_logic;

	-- Digital Signals
	signal pwm_sginal			: std_logic_vector(9 downto 0);
	
	-- Communicated Signals
	signal speed_ctrl_in : std_logic_vector(7 downto 0) := X"00";
	signal power_ctrl_in	: std_logic_vector(3 downto 0) := X"0";
	signal nes_a	: std_logic	:= '1';
	signal nes_b	: std_logic	:= '1';
	signal charge	: std_logic	:= '1';
	signal debug	: std_logic := '0';
	signal debug_phase	: std_logic_vector(1 downto 0) := "00";
	
	begin
 
-- Instantiate PLL to generate system clock
	pll : entity pll_core.pll_core(rtl)
	port map(
		refclk	=> clk_24,
		outclk_0	=>	clk_40,
		outclk_1	=>	clk_2k
	);
	 
-- The instantiation will create connect this block to the JTAG chain 
	virtual_jtag : entity vjtag.vjtag(rtl)
	port map(
		TDI 	=> tdi_sig,
		TDO 	=> tdo_sig,
		TCK	=> tck,
		IR_IN		=> ir,
		virtual_state_uir => data_ready,
		virtual_state_sdr => sdr_valid,
		virtual_state_cdr => load_dr
	 );

	tdo_sig <= dr(0);
	dr_proc: process (tck, tdi_sig, tdo_sig, load_dr, sdr_valid) begin
		if (rising_edge(tck)) then
			if (load_dr = '1') then
				if(ir = X"00") then
					dr <= X"12345678";
				elsif(ir = X"09") then
					dr <= X"87654321";
				else
					dr <= X"00000000";
				end if;
			elsif (sdr_valid = '1') then
				dr <= (tdi_sig & dr(31 downto 1));
			end if;
		end if;
	end process;
	
	pwm_proc: process (tck, data_ready) begin
		if (rising_edge(tck)) then
			if(data_ready = '1') then
				if(ir = X"01") then
					speed_ctrl_in <= dr(7 downto 0);
				elsif(ir = X"02") then
					power_ctrl_in <= dr(3 downto 0);
				elsif(ir = X"03") then
					nes_a <= dr(0);
				elsif(ir = X"04") then
					nes_b <= dr(0);
				elsif(ir = X"05") then
					charge <= dr(0);
				elsif(ir = X"06") then
					debug <= dr(0);
				elsif(ir = X"07") then
					debug_phase <= dr(1 downto 0);
				elsif(ir = X"08") then
					position <= dr(9 downto 0);
				end if;
			end if;
		end if;
	end process;
	
	
--counter : entity work.counter(rtl)
--	port map(
--		rst_n_in 	=> RST_IN,
--		clk_in 		=> clk_2k,
--		ena_in 		=> '1',
--		dir_in 		=> NES_A,
--		prescale_in	=> SPEED_CTRL_IN,
--		counter_out => position
--	 );

-- Instantiate and connect three phase generator
three_phase_lookup : entity work.three_phase(rtl)
	port map(
		-- FX2 interface
		rst_n_in    => RST_IN,
		clk_in      =>	clk_40,
		
		degree_in	=> position,
		
		sine_uh 		=> pwm_uh,
		sine_ul		=> pwm_ul,
		sine_vh 		=> pwm_vh,
		sine_vl		=> pwm_vl,
		sine_wh 		=> pwm_wh,
		sine_wl		=> pwm_wl
	);
	
	process (POWER_CTRL_IN, CHARGE, DEBUG_PHASE) begin
		if(CHARGE = '0') then
			pwm_masked_uh <= (others => '0');
			pwm_masked_ul <= "0011111111";
			pwm_masked_vh <= (others => '0');
			pwm_masked_vl <= "0011111111";
			pwm_masked_wh <= (others => '0');
			pwm_masked_wl <= "0011111111";
		elsif(DEBUG = '1') then
			if(DEBUG_PHASE = "01" and NES_A = '0') then		-- Debug Phase U Lock
				pwm_masked_uh <= "0011111000";
				pwm_masked_ul <= (others => '0');
				pwm_masked_vh <= (others => '0');
				pwm_masked_vl <= "1111111000";
				pwm_masked_wh <= (others => '0');
				pwm_masked_wl <= "1111111000";
			elsif(DEBUG_PHASE = "10" and NES_A = '0') then		-- Debug Phase V Lock
				pwm_masked_uh <= (others => '0');
				pwm_masked_ul <= "1111111000";
				pwm_masked_vh <= "0011111000";
				pwm_masked_vl <= (others => '0');
				pwm_masked_wh <= (others => '0');
				pwm_masked_wl <= "1111111000";
			elsif(DEBUG_PHASE = "11" and NES_A = '0') then		-- Debug Phase W Lock
				pwm_masked_uh <= (others => '0');
				pwm_masked_ul <= "1111111000";
				pwm_masked_vh <= (others => '0');
				pwm_masked_vl <= "1111111000";
				pwm_masked_wh <= "0011111000";
				pwm_masked_wl <= (others => '0');
			else
				pwm_masked_uh <= (others => '0');
				pwm_masked_ul <= "1111111000";
				pwm_masked_vh <= (others => '0');
				pwm_masked_vl <= "1111111000";
				pwm_masked_wh <= (others => '0');
				pwm_masked_wl <= "1111111000";
			end if;
		else
			if(POWER_CTRL_IN = X"7") then
				pwm_masked_uh <= pwm_uh(9 downto 0);
				pwm_masked_ul <= pwm_ul(9 downto 0);
				pwm_masked_vh <= pwm_vh(9 downto 0);
				pwm_masked_vl <= pwm_vl(9 downto 0);
				pwm_masked_wh <= pwm_wh(9 downto 0);
				pwm_masked_wl <= pwm_wl(9 downto 0);
			elsif(POWER_CTRL_IN = X"6") then
				pwm_masked_uh <= "0" & pwm_uh(9 downto 1);
				pwm_masked_ul <= pwm_ul(9 downto 0);
				pwm_masked_vh <= "0" & pwm_vh(9 downto 1);
				pwm_masked_vl <= pwm_vl(9 downto 0);
				pwm_masked_wh <= "0" & pwm_wh(9 downto 1);
				pwm_masked_wl <= pwm_wl(9 downto 0);
			elsif(POWER_CTRL_IN = X"5") then
				pwm_masked_uh <= "00" & pwm_uh(9 downto 2);
				pwm_masked_ul <= pwm_ul(9 downto 0);
				pwm_masked_vh <= "00" & pwm_vh(9 downto 2);
				pwm_masked_vl <= pwm_vl(9 downto 0);
				pwm_masked_wh <= "00" & pwm_wh(9 downto 2);
				pwm_masked_wl <= pwm_wl(9 downto 0);
			elsif(POWER_CTRL_IN = X"4") then
				pwm_masked_uh <= "000" & pwm_uh(9 downto 3);
				pwm_masked_ul <= pwm_ul(9 downto 0);
				pwm_masked_vh <= "000" & pwm_vh(9 downto 3);
				pwm_masked_vl <= pwm_vl(9 downto 0);
				pwm_masked_wh <= "000" & pwm_wh(9 downto 3);
				pwm_masked_wl <= pwm_wl(9 downto 0);
			elsif(POWER_CTRL_IN = X"3") then
				pwm_masked_uh <= "0000" & pwm_uh(9 downto 4);
				pwm_masked_ul <= pwm_ul(9 downto 0);
				pwm_masked_vh <= "0000" & pwm_vh(9 downto 4);
				pwm_masked_vl <= pwm_vl(9 downto 0);
				pwm_masked_wh <= "0000" & pwm_wh(9 downto 4);
				pwm_masked_wl <= pwm_wl(9 downto 0);
			elsif(POWER_CTRL_IN = X"2") then
				pwm_masked_uh <= "00000" & pwm_uh(9 downto 5);
				pwm_masked_ul <= pwm_ul(9 downto 0);
				pwm_masked_vh <= "00000" & pwm_vh(9 downto 5);
				pwm_masked_vl <= pwm_vl(9 downto 0);
				pwm_masked_wh <= "00000" & pwm_wh(9 downto 5);
				pwm_masked_wl <= pwm_wl(9 downto 0);
			elsif(POWER_CTRL_IN = X"1") then
				pwm_masked_uh <= "000000" & pwm_uh(9 downto 6);
				pwm_masked_ul <= pwm_ul(9 downto 0);
				pwm_masked_vh <= "000000" & pwm_vh(9 downto 6);
				pwm_masked_vl <= pwm_vl(9 downto 0);
				pwm_masked_wh <= "000000" & pwm_wh(9 downto 6);
				pwm_masked_wl <= pwm_wl(9 downto 0);
			else
				pwm_masked_uh <= "0000000" & pwm_uh(9 downto 7);
				pwm_masked_ul <= pwm_ul(9 downto 0);
				pwm_masked_vh <= "0000000" & pwm_vh(9 downto 7);
				pwm_masked_vl <= pwm_vl(9 downto 0);
				pwm_masked_wh <= "0000000" & pwm_wh(9 downto 7);
				pwm_masked_wl <= pwm_wl(9 downto 0);
			end if;
		end if;
	end process;

uh_pwm : entity work.pwm(rtl)
	port map(
		rst_n_in		=> RST_IN,
		clk_in		=> clk_40,
		ena_in      => '1',
		value_in		=> pwm_masked_uh,
		pwm_out     => uh_signal,
		sample_out	=> sample_signal				--  Right now this is super sketchy because I can't be positive all the PWM blocks are synchronized but let's just pretend they are... 
	);

SAMPLE_OUT <= not sample_signal;

vh_pwm : entity work.pwm(rtl)
	port map(
		rst_n_in		=> RST_IN,
		clk_in		=> clk_40,
		ena_in      => '1',
		value_in		=> pwm_masked_vh,
		pwm_out     => vh_signal
	);

wh_pwm : entity work.pwm(rtl)
	port map(
		rst_n_in		=> RST_IN,
		clk_in		=> clk_40,
		ena_in      => '1',
		value_in		=> pwm_masked_wh,
		pwm_out     => wh_signal
	);

ul_pwm : entity work.pwm(rtl)
	port map(
		rst_n_in		=> RST_IN,
		clk_in		=> clk_40,
		ena_in      => '1',
		value_in		=> pwm_masked_ul,
		pwm_out     => ul_signal
	);

vl_pwm : entity work.pwm(rtl)
	port map(
		rst_n_in		=> RST_IN,
		clk_in		=> clk_40,
		ena_in      => '1',
		value_in		=> pwm_masked_vl,
		pwm_out     => vl_signal
	);

wl_pwm : entity work.pwm(rtl)
	port map(
		rst_n_in		=> RST_IN,
		clk_in		=> clk_40,
		ena_in      => '1',
		value_in		=> pwm_masked_wl,
		pwm_out     => wl_signal
	);	

	
	-- Dead Time Block 1.5uS minimum 25nS at 40MHz so 60 minimum, 100 for good measure gives us
u_dead_time: entity work.dead_time(rtl)
	port map(
	 rst_n_in => RST_IN,
    clk_in => clk_40,
    ena_in => '1',
	 pol_in => '1',						-- Invert polarity for optcoupler circuit
    high_side_in => uh_signal,
    low_side_in => ul_signal,
    dead_time_in => "01100100",
    high_side_out => UH_OUT,
    low_side_out => UL_OUT
	);
	
v_dead_time: entity work.dead_time(rtl)
	port map(
	 rst_n_in => RST_IN,
    clk_in => clk_40,
    ena_in => '1',
	 pol_in => '1',						-- Invert polarity for optcoupler circuit
    high_side_in => vh_signal,
    low_side_in => vl_signal,
    dead_time_in => "01100100",
    high_side_out => VH_OUT,
    low_side_out => VL_OUT
	);
	
w_dead_time: entity work.dead_time(rtl)
	port map(
	 rst_n_in => RST_IN,
    clk_in => clk_40,
    ena_in => '1',
	 pol_in => '1',						-- Invert polarity for optcoupler circuit
    high_side_in => wh_signal,
    low_side_in => wl_signal,
    dead_time_in => "01100100",
    high_side_out => WH_OUT,
    low_side_out => WL_OUT
	);
	
	resolver_control <= res & address;
	
resolver: entity work.spi(rtl)
	port map(
		reset_n => RST_IN,
		clk_in => clk_40,
		trigger_reading => clk_2k,
		spi_data_o => resolver_signal,
		
		-- Test rig
		switches_in => resolver_control,
		
		-- AD2S1210 
		A => A_OUT,
		RES => RES_OUT,
		-- sample_o => SAMPLE_OUT,
		WR_N_o => WR_OUT,
		SCLK => SCLK_OUT,
		MISO => MISO_IN,
		MOSI => MOSI_OUT		
	);

--==============================================
-- Stateless Signals
--==============================================
	 leds_out <= not dr(7 downto 0);
	 
end architecture rtl;