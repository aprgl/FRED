library ieee;
use ieee.numeric_std.all;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

--============================================================================
-- Little Counter Block
--============================================================================
-- Generate a 1024 bit bidirectional counter 
-- Version: 0.0.0 Initial Commit -Shaun
------------------------------------------------------------------------------

entity counter is
Port (
    
    rst_n_in    : in    std_logic;
    clk_in      : in    std_logic;
    ena_in      : in    std_logic;
    dir_in		 : in    std_logic;
	 prescale_in : in std_logic_vector(7 downto 0);

	 counter_out	: out std_logic_vector(9 downto 0)
	 
    );
end entity counter;

architecture rtl of counter is
    
    signal the_count    : std_logic_vector(9 downto 0)  :=  (others => '0');
	 signal prescale		: std_logic_vector(7 downto 0)	:= (others => '0');

begin
    
    --========================================================================
    -- Counter Logic
    --========================================================================
    count_proc: process (clk_in, rst_n_in) begin
        if( rising_edge(clk_in) ) then
            if( rst_n_in = '0') then
                the_count <= (others => '0');
					 prescale <= (others => '0');
            end if;

				if (ena_in = '1' and prescale >= prescale_in) then
					prescale <= (others => '0');
					if (dir_in = '1') then
						 the_count <= the_count + 1;
					else
						 the_count <= the_count - 1;
					end if;
				end if;
				
				prescale <= prescale + 1;
				counter_out <= the_count;
        end if;
    end process count_proc;
end architecture rtl;