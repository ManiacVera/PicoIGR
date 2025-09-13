#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "pico/time.h"
#include <cstdio>

#define PIN_RESET 2
#define PIN_CLK 3
#define PIN_DAT 4
#define PIN_SS 5
#define PIN_CMD 6

#define DEBOUNCE_MS 150
#define POST_RESET_WAIT_MS 2000

#define BTN_SELECT   0x40  // val1 bit 0
#define BTN_START    0x08  // val1 bit 3
#define BTN_L2       0x80  // antes 0x01
#define BTN_R2       0x08  // antes 0x02
#define BTN_L1       0x10  // antes 0x04
#define BTN_R1       0x20  // antes 0x08
#define BTN_TRIANGLE 0x01  // antes 0x10
#define BTN_CIRCLE   0x02  // antes 0x20
#define BTN_CROSS    0x04  // antes 0x40
#define BTN_SQUARE   0x40  // antes 0x80


// val1 (byte 4)
#define SELECT_MASK   (1 << 6)  // bit 0
#define L3_MASK       0x02  // bit 1
#define R3_MASK       0x04  // bit 2
#define START_MASK    0x08  // bit 3
#define UP_MASK       0x10  // bit 4
#define RIGHT_MASK    0x20  // bit 5
#define DOWN_MASK     0x40  // bit 6
#define LEFT_MASK     0x80  // bit 7

// val2 (byte 5)
#define L2_MASK       0x01  // bit 0
#define R2_MASK       0x02  // bit 1
#define L1_MASK       0x04  // bit 2
#define R1_MASK       0x08  // bit 3
#define TRIANGLE_MASK 0x10  // bit 4
#define CIRCLE_MASK   0x20  // bit 5
#define CROSS_MASK    0x40  // bit 6
#define SQUARE_MASK   0x80  // bit 7


enum Action
{
	ACTION_NONE,
	ACTION_SHORT_RESET,
	ACTION_LONG_RESET
};

volatile bool ready_to_process = false;
volatile uint8_t val1 = 0xFF, val2 = 0xFF, select_byte = 0xFF;
volatile Action detected_action = ACTION_NONE;
volatile bool timer_busy = false; 

void print_bin(uint8_t b) {
    for (int i = 7; i >= 0; i--) {
        printf("%d", (b >> i) & 1);
    }
}

bool is_pressed(uint8_t val, uint8_t mask)
{
	return (val & mask) == 0;
}

Action detect_combo(uint8_t val1, uint8_t val2, uint select_byte)
{
    if (is_pressed(select_byte, BTN_SELECT) && is_pressed(val1, BTN_START) &&
        is_pressed(val2, BTN_L2) && is_pressed(val2, BTN_R2))
        return ACTION_SHORT_RESET;

    if (is_pressed(select_byte, BTN_SELECT) &&
        is_pressed(val2, BTN_L2) && is_pressed(val2, BTN_R2) && is_pressed(val2, BTN_CROSS))
        return ACTION_LONG_RESET;

    return ACTION_NONE;
}


// Action detect_combo(uint8_t val1, uint8_t val2)
// {
//     uint16_t switches = (val1 << 8) | val2;

//     if (switches == 0xFCF6) // combinación corto
//         return ACTION_SHORT_RESET;

//     if (switches == 0xBCFE) // combinación largo
//         return ACTION_LONG_RESET;

//     return ACTION_NONE;
// }

void short_reset()
{
	printf("Short reset triggered\n");
	gpio_put(PIN_RESET, 0);
	sleep_ms(100);
	gpio_put(PIN_RESET, 1);
	sleep_ms(POST_RESET_WAIT_MS);
}

void long_reset()
{
	printf("Long reset triggered\n");
	gpio_put(PIN_RESET, 0);
	sleep_ms(2000);
	gpio_put(PIN_RESET, 1);
	sleep_ms(POST_RESET_WAIT_MS);
}

bool wait_for_pin_state(uint pin, int expected_state, uint32_t timeout_ms) {
    absolute_time_t start = get_absolute_time();
    while (gpio_get(pin) != expected_state) {
        if (absolute_time_diff_us(get_absolute_time(), start) > timeout_ms * 1000) {
            return false; // timeout
        }
    }
    return true; // cambio detectado
}

uint8_t transfer_byte_slave()
{
    uint8_t in_byte = 0;

    for (int i = 0; i < 8; i++)
    {
        if (!wait_for_pin_state(PIN_CLK, 1, 100)) {
            printf("Timeout esperando CLK sube\n");
            break;
        }

        bool bit_in = gpio_get(PIN_DAT);
        if (bit_in)
            in_byte |= (1 << i);

        if (!wait_for_pin_state(PIN_CLK, 0, 100)) {
            printf("Timeout esperando CLK baja\n");
            break;
        }
    }

    return in_byte;
}

uint8_t reverse_byte(uint8_t b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

void read_controller_slave(uint8_t &val1, uint8_t &val2, uint8_t &select_byte)
{
    if (!wait_for_pin_state(PIN_SS, 0, 100)) {
        return;
    }

    // Espera sincronización con primer pulso de CLK
    if (!wait_for_pin_state(PIN_CLK, 1, 100)) {
        return;
    }

    // Lee datos desde el principio real de la transacción
    uint8_t b0 = transfer_byte_slave(); // 0x01
    uint8_t b1 = transfer_byte_slave(); // 0x42
    uint8_t b2 = transfer_byte_slave(); // 0x5A
    uint8_t b3 = transfer_byte_slave(); // botones alto
    uint8_t b4 = transfer_byte_slave(); // botones bajo
    uint8_t b5 = transfer_byte_slave(); // opcional

    if (!wait_for_pin_state(PIN_SS, 1, 100)) {
        return;
    }

    // printf("b3: ");
    // print_bin(b3);
    // printf("\n");
    // printf("  b4: ");
    // print_bin(b4);
    // printf("\n");
    // printf("b2: ");
    // print_bin(b2);
    // printf("\n");

    // b3 = reverse_byte(b3);
    // b4 = reverse_byte(b4);

    // printf("SPI Raw Bytes: %02X %02X %02X %02X %02X %02X\n", b0, b1, b2, b3, b4, b5);
    printf("SPI Raw Bytes: %02X %02X %02X %02X %02X %02X\n",
            reverse_byte(b0), reverse_byte(b1), reverse_byte(b2),
            reverse_byte(b3), reverse_byte(b4), reverse_byte(b5));

    val1 = b3;
    val2 = b4;
    select_byte = b2;
    sleep_ms(500);
}

bool timer_callback(repeating_timer_t *rt)
{
	uint32_t status = save_and_disable_interrupts();  // ← entra sección crítica

    if (timer_busy) {
        restore_interrupts(status);
        return true;
    }

    timer_busy = true;
    restore_interrupts(status);  // ← sale de sección crítica
    uint8_t v1, v2, select_byte;
    read_controller_slave(v1, v2, select_byte);

    Action action = detect_combo(v1, v2, select_byte);

    if (action != ACTION_NONE) {
        sleep_ms(DEBOUNCE_MS);

        uint8_t v1_confirm, v2_confirm, select_confirm;
        read_controller_slave(v1_confirm, v2_confirm, select_confirm);

        if (detect_combo(v1_confirm, v2_confirm, select_confirm) == action) {
            val1 = v1_confirm;
            val2 = v2_confirm;
            select_byte = select_confirm;
            detected_action = action;
            ready_to_process = true;
        }
    }

    return true;
}



void print_buttons(uint8_t val1, uint8_t val2) {
    if (is_pressed(select_byte, BTN_SELECT)) printf("Select ");
    if (is_pressed(val1, BTN_START)) printf("Start ");

    if (is_pressed(val2, BTN_L2)) printf("L2 ");
    if (is_pressed(val2, BTN_R2)) printf("R2 ");
    if (is_pressed(val2, BTN_L1)) printf("L1 ");
    if (is_pressed(val2, BTN_R1)) printf("R1 ");
    if (is_pressed(val2, BTN_TRIANGLE)) printf("Triángulo ");
    if (is_pressed(val2, BTN_CIRCLE)) printf("Círculo ");
    if (is_pressed(val2, BTN_CROSS)) printf("X ");
    if (is_pressed(val2, BTN_SQUARE)) printf("Cuadro ");

    printf("val1 bits: ");
    print_bin(val1);
    printf(" | val2 bits: ");
    print_bin(val2);
    printf(" | select bits: ");
    print_bin(select_byte);
    printf("\n");
}


bool doMain()
{
    uint8_t v1, v2, select_byte;
    read_controller_slave(v1, v2, select_byte);
    print_buttons(v1, v2);
    Action action = detect_combo(v1, v2, select_byte);

    if (action != ACTION_NONE) {
        sleep_ms(DEBOUNCE_MS);

        uint8_t v1_confirm, v2_confirm, select_confirm;
        read_controller_slave(v1_confirm, v2_confirm, select_confirm);

        if (detect_combo(v1_confirm, v2_confirm, select_confirm) == action) {
            val1 = v1_confirm;
            val2 = v2_confirm;
            select_byte = select_confirm;
            detected_action = action;
            ready_to_process = true;
        }
    }

    return true;
}

int main()
{
	stdio_init_all();
	sleep_ms(2000);

	// PIN_RESET: salida, control del reset del mando
	gpio_init(PIN_RESET);
	gpio_set_dir(PIN_RESET, GPIO_OUT);
	gpio_put(PIN_RESET, 1);  // Normalmente alto (activo en bajo)

	// PIN_CLK: ENTRADA, la consola manda el reloj
	gpio_init(PIN_CLK);
	gpio_set_dir(PIN_CLK, GPIO_IN);
	gpio_pull_up(PIN_CLK);  // Pull-up opcional para evitar flote

	// PIN_SS: ENTRADA, la consola activa el mando
	gpio_init(PIN_SS);
	gpio_set_dir(PIN_SS, GPIO_IN);
	gpio_pull_up(PIN_SS);
	
	gpio_init(PIN_DAT);
	gpio_set_dir(PIN_DAT, GPIO_IN);
	gpio_pull_up(PIN_DAT);

	// PIN_CMD: ENTRADA, la consola manda comandos
	gpio_init(PIN_CMD);
	gpio_set_dir(PIN_CMD, GPIO_IN);
	gpio_pull_up(PIN_CMD);

	// // Timer
	//repeating_timer_t timer;
	//add_repeating_timer_ms(120, timer_callback, NULL, &timer);

	while (true)
	{	
		doMain();

		if (ready_to_process)
		{
			ready_to_process = false;

			switch (detected_action)
			{
			case ACTION_SHORT_RESET:
				short_reset();
				break;
			case ACTION_LONG_RESET:
				long_reset();
				break;
			default:
				break;
			}
		}
		sleep_ms(1);
	}

	return 0;
}
