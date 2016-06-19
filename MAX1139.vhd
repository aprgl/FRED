library ieee;
use ieee.numeric_std.all;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

--============================================================================
--  Phase Currents from MAX1139
--============================================================================
-- Driver for the MAX1139 ADC for reading motor phase currents.
-- Version: 0.0.1 - Actually cleaning up the documentation. Trying to get this
--     code to a working level. There's a bit on nonsense in here. -Shaun
------------------------------------------------------------------------------

entity MAX1139 is
Port (
	-- clock and reset signals
	clk_in			: in	std_logic;
	rst_n_in		: in	std_logic;

	-- I2C data and flow control
	data_rdy_out	: out	std_logic;
	tx_data_out		: out	std_logic_vector(7 downto 0);
	rx_data_in		: in	std_logic_vector(7 downto 0);
	i2c_busy_in		: in	std_logic;
	rw_out			: out 	std_logic;

	-- motor phase currents
	u_current_out	: out	std_logic_vector(9 downto 0);
	v_current_out	: out	std_logic_vector(9 downto 0);
	w_current_out	: out	std_logic_vector(9 downto 0);
	
	-- trigger reading
	trigger_in		: in	std_logic

	);
end entity MAX1139;

architecture rtl of MAX1139 is
	
	signal adc_reg		: std_logic_vector(3 downto 0) := (Others => '0');
	signal u_current	: std_logic_vector(9 downto 0) := (Others => '0');
	signal v_current	: std_logic_vector(9 downto 0) := (Others => '0');
	signal w_current	: std_logic_vector(9 downto 0) := (Others => '0');
	 
	-- State Machine Signals
	type state_type is (state_reset,
							state_setup,
							s_hold1,
							state_config,
							s_hold2,
							state_read,
							s_hold3,
							state_done);
	signal state, next_state: state_type := state_reset; -- legal?

	signal i2c_busy_last	: std_logic;
	
	begin

	--========================================================================
	--  State Machine Control
	--========================================================================
	-- State machine control block - reset and next state indexing
	--------------------------------------------------------------------------
	
	-- State machine control block - reset and next state indexing
	state_machine_ctrl: process (rst_n_in, clk_in) begin
	if (rising_edge(clk_in)) then
		if (rst_n_in = '0') then
			state <= state_reset;   -- default state on reset
		else
			state <= next_state;	-- clocked change of state
		end if;
	end if;
	end process state_machine_ctrl;

	-- State machine reading control block - need to read multiple registers
	state_machine_read_ctrl: process (rst_n_in, clk_in, i2c_busy_in, i2c_busy_last, state) begin
	if (rising_edge(clk_in)) then
		if (state = state_reset) then
			adc_reg <= (Others => '0');
			u_current <= (Others => '0');
			v_current <= (Others => '0');
			w_current <= (Others => '0');
		elsif (state = s_hold3) then
			i2c_busy_last <= i2c_busy_in; -- pulled from eewiki's spi to i2c code
			if (i2c_busy_last = '1' and i2c_busy_in = '0') then	-- Hot&Fresh Data
				adc_reg <= adc_reg + '1';
				if (adc_reg = "0000") then
					u_current(9 downto 8) <= rx_data_in(1 downto 0);
				elsif (adc_reg = "0001") then
					u_current(7 downto 0) <= rx_data_in;
				elsif (adc_reg = "0010") then
					v_current(9 downto 8) <= rx_data_in(1 downto 0);
				elsif (adc_reg = "0011") then
					v_current(7 downto 0) <= rx_data_in;
				elsif (adc_reg = "0100") then
					w_current(9 downto 8) <= rx_data_in(1 downto 0);
				elsif (adc_reg = "0101") then
					w_current(7 downto 0) <= rx_data_in;
				end if;
			end if;
		elsif (state = state_done) then
			u_current_out <= u_current;
			v_current_out <= v_current;
			w_current_out <= w_current;
		end if;
	end if;
	end process state_machine_read_ctrl;
	
	-- State machine
	state_machine: process (state, i2c_busy_in, adc_reg, trigger_in) begin
	case state is
		-- In a reset state -- reassert configuration registers.
		when state_reset =>
			if ( i2c_busy_in = '0' and trigger_in = '1') then		-- If the I2C lines are free
				next_state <= state_setup;
			else
				next_state <= state_reset;
			end if;

			-- OUTPUTS --			
			tx_data_out <= (Others => '0');
			data_rdy_out <= '0';
			rw_out <= '0';					-- Writing
			
		when state_setup =>
			if (i2c_busy_in = '0') then
				next_state <= state_setup;
			else
				next_state <= s_hold1;
			end if;

			-- OUTPUTS --
			-- REG SEL2 SEL1 SEL0 CLK BIP/UNI RST X
			-- REG = 1 - Setup Register
			-- SEL[2:0] = (5 - Internal Reference Voltage (2.046))
			-- CLK = 0 - Internal Clock
			-- BIP/UNI = 0 - Unipolar Inputs
			-- RST = 1 - Don't Reboot!
			-- X = 0 - Don't Care Bit
			tx_data_out <=  "11010010";
			data_rdy_out <= '1';
			rw_out <= '0';					-- Writing
					
		when s_hold1 =>
			if (i2c_busy_in = '1') then
				next_state <= s_hold1;
			else
				next_state <= state_config;
			end if;

			-- OUTPUTS --
			tx_data_out <=  "10000010";
			data_rdy_out <= '0';
			rw_out <= '0';					-- Writing
		
		when state_config =>
			if (i2c_busy_in = '0') then
				next_state <= state_config;
			else
				next_state <= s_hold2;
			end if;
			
			-- OUTPUTS --
			-- REG SCAN1 SCAN0 CS3 CS2 CS1 CS0 SGL/DIF
			-- REG = 0 - Configuration Register
			-- SCAN[1:0] = 0 - Scan from A0 to A2
			-- CS[3:0] = 2 - Set the upper bound of the scan at Ch2
			-- SGL/DIF = 1 - Single Ended			
			tx_data_out <=  "00000101";
			data_rdy_out <= '1';
			rw_out <= '0';					-- Writing
			
		when s_hold2 =>
			if (i2c_busy_in = '1') then
				next_state <= s_hold2;
			else
				next_state <= state_read;
			end if;

			-- OUTPUTS --
			tx_data_out <= (Others => '0');
			data_rdy_out <= '0';
			rw_out <= '0';					-- Writing
			
		
		when state_read =>
			if (i2c_busy_in = '0') then
				next_state <= state_read;
			else
				next_state <= s_hold3;
			end if;
			
			-- OUTPUTS --
			tx_data_out <= (Others => '0');
			data_rdy_out <= '1';
			rw_out <= '1';					-- Writing
			
		when s_hold3 =>
			if (i2c_busy_in = '1') then
				next_state <= s_hold3;
			elsif (adc_reg < "0101") then
				next_state <= state_read;
			else
				next_state <= state_done;
			end if;

			-- OUTPUTS --
			tx_data_out <= (Others => '0');
			data_rdy_out <= '1';
			rw_out <= '1';					-- Writing
			
		when state_done =>
			if(trigger_in = '0') then		-- If the I2C lines are free
				next_state <= state_reset;
			else
				next_state <= state_done;
			end if;

			-- OUTPUTS --			
			tx_data_out <= (Others => '0');
			data_rdy_out <= '0';
			rw_out <= '0';					-- Writing

		end case;
	end process state_machine;
	-------------------------------------------------------------------------
	
	--=======================================================================
	--  Stateless Signals
	--=======================================================================

	-------------------------------------------------------------------------
	
	end architecture rtl;
