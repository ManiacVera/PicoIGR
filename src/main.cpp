#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/structs/iobank0.h"
#include "config.h"
#include "pad.h"
#include "psxSPI.pio.h"
#include <cstdio>
#include <string.h>

// #define FORCE_PAD
#define PAD_TOP 0x01
#define PAD_READ 0x42
#define SECONDS 3
#define COMBINATION_CANCEL R1

uint smCmdReader;
uint smDatReader;

#ifdef FORCE_PAD
uint smCmdWriter;
#endif

#define RECV_CMD() read_byte_blocking(pio0, smCmdReader)
#define RECV_DAT() read_byte_blocking(pio0, smDatReader)

#ifdef FORCE_PAD
#define SEND_CMD(byte) write_byte_blocking(pio0, smCmdWriter, byte)
#endif

enum Action
{
	ACTION_NONE,
	ACTION_SHORT_RESET,
	ACTION_LONG_RESET,
	ACTION_MENU_RESET,
	ACTION_DOOR
};

enum DoorState
{
	DOOR_OPEN,
	DOOR_CLOSED
};

volatile absolute_time_t cancel_start_time = 0;
volatile bool cancel_time_active = true;
volatile uint16_t first_button = 0;
volatile int first_button_pressed = -1;
volatile bool ready_to_process = false;
volatile Action detected_action = ACTION_NONE;
volatile DoorState door_state = DOOR_CLOSED;
uint offsetCmdReader;
uint offsetDatReader;
uint offsetCmdWriter;
const uint8_t id_data[] = {0x04, 0x00, 0x00, 0x80};

void restar_variables_time()
{
	ready_to_process = false;
	cancel_start_time = get_absolute_time();	
	cancel_time_active = true;
	first_button = 0;
	first_button_pressed = -1;
}

void close_door(bool wait)
{
	gpio_put(PIN_LED, 0);

	// printf("CLOSING\n");
	door_state = DOOR_CLOSED;
	gpio_put(PIN_DOOR, 0);
	gpio_set_dir(PIN_DOOR, GPIO_OUT);

	if (wait)
		sleep_ms(700);
}

void open_door(bool wait)
{
	gpio_put(PIN_LED, 1);

	// printf("OPENING\n");
	door_state = DOOR_OPEN;
	gpio_set_dir(PIN_DOOR, GPIO_IN);

	if (wait)
		sleep_ms(700);
}

void short_reset()
{
	close_door(false);
	gpio_put(PIN_RESET, 0);
	sleep_ms(700);
	gpio_put(PIN_RESET, 1);
	restar_variables_time();
}

void long_reset()
{
	close_door(false);
	gpio_put(PIN_RESET, 0);
	sleep_ms(2000);
	gpio_put(PIN_RESET, 1);
	restar_variables_time();
}

void menu_reset()
{
	open_door(false);
	gpio_put(PIN_RESET, 0);
	sleep_ms(2000);
	gpio_put(PIN_RESET, 1);
	restar_variables_time();
}

void simulate_door()
{
	if (door_state == DOOR_OPEN)
	{
		close_door(true);
	}
	else
	{
		open_door(true);
	}
}

// This code is an attempt to force the console to send button presses on the controller but it gets stuck.
void force_controller_response(void) {
#ifdef FORCE_PAD
    // Configure SEL and CLK pins as outputs and set initial idle states
    gpio_init(PIN_SEL);
    gpio_set_dir(PIN_SEL, GPIO_OUT);
    gpio_put(PIN_SEL, 0);   // Deassert ATT (SEL high)

    gpio_init(PIN_CLK);
    gpio_set_dir(PIN_CLK, GPIO_OUT);
    gpio_put(PIN_CLK, 1);  // CLK idle high

	// Setup CMD pin as output (important!)
    gpio_init(PIN_CMD);
    gpio_set_dir(PIN_CMD, GPIO_OUT);
    gpio_put(PIN_CMD, 1);

    // Enable state machines for command writing and data reading
    pio_sm_set_enabled(pio0, smCmdWriter, true);
    pio_sm_set_enabled(pio0, smDatReader, true);

    absolute_time_t start_time = get_absolute_time();

    while (!ready_to_process && absolute_time_diff_us(start_time, get_absolute_time()) < SECONDS * 1000000) {
        gpio_put(PIN_SEL, 0);  // Assert ATT (SEL low)
        sleep_us(10);          // Small delay for signal stabilization


        // Send command sequence to request controller button data
        SEND_CMD(0x01);        // Start byte
        SEND_CMD(0x42);        // PAD_READ command
        SEND_CMD(0x00);        // Placeholder byte		
		printf("HERE STUCK\n");

		// Clear FIFO in data reader state machine before reading
		pio_sm_clear_fifos(pio0, smDatReader);

        // Read two bytes of controller button data from FIFO
		uint16_t sw_status = RECV_DAT();
		sw_status |= RECV_DAT() << 8;

		switch (sw_status)
		{
			case COMBINATION_CANCEL:
				detected_action = ACTION_DOOR;
				ready_to_process = true;
			break;
			default:
			break;
		}
		

        gpio_put(PIN_SEL, 1);  // Deassert ATT (SEL high)
        sleep_ms(100);         // Wait before next attempt
    }

    // Disable the state machines after timeout to release the bus
    pio_sm_set_enabled(pio0, smCmdWriter, false);
    pio_sm_set_enabled(pio0, smDatReader, false);

    // Set data pins back to inputs to avoid bus contention
    gpio_set_dir(PIN_CMD, GPIO_IN);
    gpio_set_dir(PIN_SEL, GPIO_IN);
    gpio_set_dir(PIN_CLK, GPIO_IN);
    gpio_set_dir(PIN_DAT, GPIO_IN);

#endif
}

void process_pad_cmd()
{
	if (RECV_CMD() != PAD_READ) // only interested in PSX trying to read pad
		return;

	RECV_CMD();							   // ignore TAP byte
	pio_sm_clear_fifos(pio0, smDatReader); // clear out Hi-Z, idlo, and idhi bytes
	uint16_t sw_status = RECV_DAT();
	sw_status |= RECV_DAT() << 8;

	if (cancel_time_active)
	{
		if (absolute_time_diff_us(cancel_start_time, get_absolute_time()) > SECONDS * 1000000) 
		{
			cancel_time_active = false;
		}
		else if (sw_status != 0xFFFF && first_button_pressed == -1) 
		{
			first_button = sw_status;
			first_button_pressed = 1;
		}
	}

	switch (sw_status)
	{
	case START &SELECT &L2 &R2:
		detected_action = ACTION_SHORT_RESET;
		ready_to_process = true;
		break;

	case SELECT &L2 &R2 &X:
		detected_action = ACTION_LONG_RESET;
		ready_to_process = true;
		break;

	case SELECT &L2 &R2 &R1:
		detected_action = ACTION_DOOR;
		ready_to_process = true;
		break;

	case SELECT &L2 &R2 &SQUARE:
		detected_action = ACTION_MENU_RESET;
		ready_to_process = true;
		break;

	case COMBINATION_CANCEL:
		if (cancel_time_active && first_button_pressed && (first_button & COMBINATION_CANCEL))
		{
			detected_action = ACTION_DOOR;
			first_button_pressed = 0;
			ready_to_process = true;
		}
		break;

	default:
		break;
	}
}

void process_cmd(uint8_t cmd)
{
	switch (cmd)
	{
	case PAD_TOP:
		process_pad_cmd();
		break;
	default:
		break;
	}
}

_Noreturn void igr_thread()
{
	while (true)
	{
		process_cmd(RECV_CMD());
	}
}

void __time_critical_func(restart_pio_sm)(void)
{
	pio_set_sm_mask_enabled(pio0, 1 << smCmdReader | 1 << smDatReader, false);
	pio_restart_sm_mask(pio0, 1 << smCmdReader | 1 << smDatReader);
	pio_sm_exec(pio0, smCmdReader, pio_encode_jmp(offsetCmdReader)); // restart smCmdReader PC
	pio_sm_exec(pio0, smDatReader, pio_encode_jmp(offsetDatReader)); // restart smDatReader PC
	pio_sm_clear_fifos(pio0, smCmdReader);
	pio_sm_clear_fifos(pio0, smDatReader);

	// resetting and launching core1 here allows to perform the reset of the transaction (e.g. when PSX polls for new MC without completing the read)
	multicore_reset_core1();
	multicore_fifo_clear_irq();
	multicore_launch_core1(igr_thread);
	pio_enable_sm_mask_in_sync(pio0, 1 << smCmdReader | 1 << smDatReader);
}

void __time_critical_func(sel_isr_callback())
{
	check_gpio_param(PIN_SEL);
	iobank0_hw->intr[PIN_SEL / 8] = GPIO_IRQ_EDGE_RISE << (4 * (PIN_SEL % 8));

	restart_pio_sm();
}

void init_pio()
{
    // Inicializa pines usados como salidas y entradas
    gpio_init(PIN_RESET);
    gpio_set_dir(PIN_RESET, GPIO_OUT);
    gpio_put(PIN_RESET, 1);

    gpio_init(PIN_DOOR);
    gpio_put(PIN_DOOR, 0);
    gpio_set_dir(PIN_DOOR, GPIO_OUT);

    gpio_init(PIN_LED);
    gpio_put(PIN_LED, 0);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    gpio_set_dir(PIN_DAT, false);
    gpio_set_dir(PIN_CMD, false);
    gpio_set_dir(PIN_SEL, false);
    gpio_set_dir(PIN_CLK, false);

    gpio_disable_pulls(PIN_DAT);
    gpio_disable_pulls(PIN_CMD);
    gpio_disable_pulls(PIN_SEL);
    gpio_disable_pulls(PIN_CLK);

    // Reclama 3 state machines en PIO0 para cmd_reader, dat_reader y cmd_writer
    smCmdReader = pio_claim_unused_sm(pio0, true);
    smDatReader = pio_claim_unused_sm(pio0, true);

#ifdef FORCE_PAD
    smCmdWriter = pio_claim_unused_sm(pio0, true);
#endif

    // Carga los 3 programas en la memoria del PIO0 y guarda sus offsets
    offsetCmdReader = pio_add_program(pio0, &cmd_reader_program);
    offsetDatReader = pio_add_program(pio0, &dat_reader_program);

#ifdef FORCE_PAD
    offsetCmdWriter = pio_add_program(pio0, &cmd_writer_program);
#endif

    // Inicializa cada SM con su programa y offset correspondiente
    cmd_reader_program_init(pio0, smCmdReader, offsetCmdReader);
    dat_reader_program_init(pio0, smDatReader, offsetDatReader);

#ifdef FORCE_PAD
    cmd_writer_program_init(pio0, smCmdWriter, offsetCmdWriter);
#endif
}

_Noreturn int simulate_igr()
{
	init_pio();

	/* Setup SEL interrupt on GPIO */
	gpio_set_irq_enabled(PIN_SEL, GPIO_IRQ_EDGE_RISE, true);
	irq_set_exclusive_handler(IO_IRQ_BANK0, sel_isr_callback); // instead of normal gpio_set_irq_callback() which has slower handling
	irq_set_enabled(IO_IRQ_BANK0, true);

	/* Setup additional GPIO configuration options */
	gpio_set_slew_rate(PIN_DAT, GPIO_SLEW_RATE_FAST);
	gpio_set_drive_strength(PIN_DAT, GPIO_DRIVE_STRENGTH_12MA);

	force_controller_response();

	/* SMs are automatically enabled on first SEL reset */
	// /* Launch memory card thread */
	// printf("Starting simulation core...");
	cancel_start_time = get_absolute_time();
	multicore_launch_core1(igr_thread);
	// printf("  done\n");

	while (true)
	{
		if (ready_to_process)
		{
			// printf("ready_to_process\n");
			ready_to_process = false;

			switch (detected_action)
			{
			case ACTION_SHORT_RESET:
				short_reset();
				break;
			case ACTION_LONG_RESET:
				long_reset();
				break;
			case ACTION_MENU_RESET:
				menu_reset();
				break;
			case ACTION_DOOR:
				simulate_door();
				break;
			default:
				break;
			}

			detected_action = ACTION_NONE;
		}
	}
}

int main()
{
	stdio_init_all();
	//sleep_ms(2000);
	simulate_igr();
}