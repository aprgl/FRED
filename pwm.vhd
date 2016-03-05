library ieee;
use ieee.numeric_std.all;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

--============================================================================
--  PWM Block
--============================================================================
-- Generate a 1024 bit PWM. 
-- Version: 0.0.0 Initial Commit - centerd PWM -Shaun
------------------------------------------------------------------------------

entity pwm is
Port (
    rst_n_in    : in    std_logic;
    clk_in      : in    std_logic;
    ena_in      : in    std_logic;
    value_in    : in    std_logic_vector(9 downto 0);
    pwm_out     : out   std_logic;
	 sample_out	 : out	std_logic; 	-- Windows for resolver sampling

    -- Signals for test bench
    pwm_counter_out : out std_logic_vector(9 downto 0);
    pwm_clk     : out   std_logic

    );
end entity pwm;

architecture rtl of pwm is
    
    signal pwm_counter  : std_logic_vector(9 downto 0)  :=  "11"&X"FF";
    signal pwm_rising   : std_logic;
    signal pwm_signal   : std_logic;

    begin
    
    --========================================================================
    --  PWM Counter Logic
    --========================================================================
    pwm_gen_proc: process (clk_in, rst_n_in) begin
        if( rising_edge(clk_in) ) then
            if( rst_n_in = '0') then
                pwm_counter <= "11"&X"FF";
                pwm_rising <= '0';
            elsif(pwm_counter = X"001" and pwm_rising = '0') then
                pwm_rising <= '1';
            elsif(pwm_counter = "11"&X"FE" and pwm_rising = '1') then
                pwm_rising <= '0';
            end if;
				
				if (pwm_rising = '1' and (pwm_counter > "11"&X"EE" and pwm_counter < "11"&X"FE")) then
					sample_out <= '1';
				else
					sample_out <= '0';
				end if;

            -- Creating the triangular waveform
            if (pwm_rising = '1') then
                pwm_counter <= pwm_counter + 1;     
            else
                pwm_counter <= pwm_counter - 1;
            end if;

            if ((rst_n_in = '0') or (pwm_counter > value_in)) then
                pwm_signal <= '0';
            elsif (pwm_counter < value_in) then
                pwm_signal <= '1';
            end if;

        end if;
    end process pwm_gen_proc;

    pwm_counter_out <= pwm_counter;
    pwm_clk <= not pwm_rising;

    --=======================================================================
    --  Stateless Signals
    --=======================================================================
    pwm_out <= '1' when (pwm_signal and ena_in) = '1' else '0';
    --------------------------------------------------------------------------
    
    end architecture rtl;