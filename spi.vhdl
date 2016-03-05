-- SPI Library by Shaun

Library ieee;
use ieee.numeric_std.all;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

entity spi is
--	generic(
--		PACKETSIZE		:	std_logic_vector(7 downto 0)	:=	X"08"
--		);
	port(
		reset_n			:	in 		std_logic;
		clk_in 			:	in 		std_logic;
		trigger_reading	:	in 		std_logic;
		spi_data_o			:	out 	std_logic_vector(23 downto 0);
		
		-- Test rig
		switches_in		:	in		std_logic_vector(3 downto 0);
		
		-- AD2S1210 
		A				:	out	std_logic_vector(1 downto 0);
		RES 			:	out	std_logic_vector(1 downto 0);
		sample_o		:	out	std_logic;
		WR_N_o 		:	out	std_logic;
		SCLK 			:	out 	std_logic;
		MISO 			:	in 	std_logic;
		MOSI 			:	out	std_logic;
		
		-- Test Signals
		counter_out	:	out	std_logic_vector(3 downto 0)
		);
end entity spi;

architecture rtl of spi is

	-- Clock Signals
    signal sys_clk			: std_logic;
    signal pwm_clk_counter	: std_logic_vector(3 downto 0);

    -- State Machine Signals
	type state_type is (state_reset, state_sample, s0, s1, s2);
	signal state, next_state: state_type;

	-- SPI Data Signals
	signal spi_enable	:	std_logic	:= '0';
	signal WR_N 		:	std_logic 	:= '1';
	signal sample 		:	std_logic	:= '0';
	signal spi_rx_data 	: std_logic_vector(23 downto 0)		:= (Others => '0');
	signal spi_tx_data 	: std_logic_vector(7 downto 0)		:= X"21";
	signal count 		:	std_logic_vector(4 downto 0)	:= (Others => '0');
	signal spi_transaction_done	: std_logic := '0';

	-- AD2S1210 config Signals
	-- Configure Address: 0x91 Value: 0x10
	signal setup 		: std_logic 	:= '0';
	signal config_count	: std_logic_vector (1 downto 0);
	--signal sample : std_logic;

begin

	-- Stateless Signals
	--A 	<= switches_in(1 downto 0);
	RES	<= switches_in(3 downto 2);
	
    --=============================================================================================
    --  CLOCK GENERATION
    --=============================================================================================
    -- Generate the SPI base clock (up to 25MHz for the AD2S1210 with VDRIVE set to 3.3V)
    -- TODO: make the clock frequency a variable. -Shaun
	-----------------------------------------------------------------------------------------------
	pwm_clk_gen_proc: process (clk_in, reset_n) begin
		if(reset_n = '0') then
			sys_clk <= '0';
			pwm_clk_counter <= (others => '0');
		elsif( rising_edge(clk_in) ) then
			if(pwm_clk_counter = X"3") then
				pwm_clk_counter <= (others => '0');
				sys_clk <= not sys_clk;
			else
				pwm_clk_counter <= pwm_clk_counter + 1;
			end if;
		end if;
    end process pwm_clk_gen_proc;
	 
	 SCLK <= sys_clk;
	 counter_out <= pwm_clk_counter;
    -----------------------------------------------------------------------------------------------
	 
	 --=============================================================================================
    --  SPI Controller for the AD2S1210
    --=============================================================================================
    -- Generate the SPI base state machine and control for SPI
    -- TODO: so much. -Shaun
	 -----------------------------------------------------------------------------------------------
	process (reset_n, sys_clk) begin
		if (reset_n = '0') then
			state <= state_reset;  				-- default state on reset
		elsif (rising_edge(sys_clk) ) then
			state <= next_state;   				-- clocked state change
		end if;
	end process;
	
	process (reset_n, state, sys_clk) begin
		if(reset_n = '0' or state = state_sample) then
			count <= (Others => '0');
		elsif (rising_edge(sys_clk) and state = s1) then
			count <= count + 1;
		end if;
	end process;

	process (reset_n, sys_clk, sample, spi_enable, spi_transaction_done) begin
		if(reset_n = '0') then
			spi_rx_data <= (Others => '0');
			spi_tx_data <= X"91";				--  Frequency Configuration Address
			setup <= '1';
			A <= "11";
			config_count <= (others => '0');
			MOSI <= '1';
		elsif(( setup = '1' and spi_transaction_done = '1') or -- Setup Mode
				sample = '1' or 
				spi_enable = '1' or 
				spi_transaction_done = '1'
				) then		-- Preload MSB
			if rising_edge(sys_clk) then
				if(spi_transaction_done = '1') then
					if(setup = '1') then
						if(config_count <= "01") then
							spi_tx_data <= X"10";				--  Frequency Configuration Address
						elsif(config_count <= "10") then
							MOSI <= '0';
							A <= "00";
							setup <= '0';
						end if;
						config_count <= config_count + 1;
					end if;
					spi_data_o <= spi_rx_data;
				else
					spi_rx_data <= spi_rx_data(22 downto 0) & MISO;		-- Shift In
					MOSI <= spi_tx_data(6);										-- MSB already shifted out
					spi_tx_data <= spi_tx_data(6 downto 0) & '0';		-- Shift Out 
				end if;
			end if;
		elsif(setup = '0') then
			MOSI <= '0';
		end if;
	end process;

	process (reset_n, sys_clk, sample) begin
		if(reset_n = '0' or sample = '0') then
			sample_o <= '1';
		elsif(sample = '1') then
			if falling_edge(sys_clk) then
				sample_o <= '0';
			end if;
		end if;
	end process;

	process (reset_n, sys_clk, WR_N) begin
		if(reset_n = '0') then
			WR_N_o <= '1';
		elsif falling_edge(sys_clk) then
			WR_N_o <= WR_N;
		end if;
	end process;

	spi: process (trigger_reading, state, spi_rx_data, spi_tx_data, MISO, count, setup) begin
		case state is
			when state_reset =>
				spi_transaction_done <= '0';
				next_state <= state_sample;  		-- default state on reset
				WR_N <= '1';
				spi_enable <= '0';
				sample <= '0';
			when state_sample =>
				if(setup = '1') then
					spi_transaction_done <= '0';
					spi_enable <= '0';
					WR_N <= '1';
					sample <= '0';					-- we're in setup mode
					next_state <= s0;
				elsif(trigger_reading = '0') then
					spi_transaction_done <= '0';
					spi_enable <= '0';
					WR_N <= '1';
					sample <= '1';					-- Trigger a sample
					next_state <= s0;
				else
					spi_transaction_done <= '0';
					spi_enable <= '0';
					WR_N <= '1';
					sample <= '0';
					next_state <= state_sample;
				end if;

			when s0 =>
				spi_transaction_done <= '0';
				spi_enable <= '0';
				wr_n <= '0';						-- Synch the datastream
				sample <= '0';
				next_state <= s1;

			when s1 =>
				spi_transaction_done <= '0';
				wr_n <= '0';						-- Synch the datastream
				sample <= '0';
				spi_enable <= '1';
				-- Two Modes Normal (a = 00 - 24 bits) and Config (a = 11 - 8 bits)
				if((setup = '1' and count = X"07") or (count = X"17")) then 
					next_state <= s2;
				else
					next_state <= s1;
				end if;

			when s2 =>
				spi_transaction_done <= '1';
				spi_enable <= '0';
				sample <= '0';
				wr_n <= '1';
				next_state <= state_sample;			-- Done, wait to sample again
		end case;
	end process;
	
end architecture rtl;