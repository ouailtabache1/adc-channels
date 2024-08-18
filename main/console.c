/*
 * console.c
 *
 *  Created on: Dec 22, 2018
 *      Author: zamek
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "console.h"
#include "cmd_system.h"

#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL ESP_LOG_ERROR
#include "esp_log.h"

#define TAG "console"

/**
 *
 */
#define CONSOLE_TASK_SIZE (4096)

#define CONSOLE_HIST_LINE_SIZE (100)

#if (CONSOLE_TASK_SIZE < 4096)
#error "Console task size must be at least 4096"
#endif

static TaskHandle_t tsk_console=NULL;
static StaticTask_t buf_console;
static StackType_t stack_console[4096];


/**
 * @addtogroup console
 * @{
 * @var G_LOG_ON
 * @brief Global variable to switch on/off logging
 *
 * Default value is true
 *
 * Command to switch on is:

 * @code
 * log on
 * @endcode

 * Command to switch on is:

 * @code
 * log off
 * @endcode

 * Asking current status is:

 * @code
 * log off
 * @endcode
 * @}
 */
bool G_LOG_ON=true;

void console_func(void *arg) {
	const char *prompt = LOG_COLOR_I "esp32> " LOG_RESET_COLOR;
	printf("\n"
	           "This is an ESP-IDF console component.\n"
	           "Type 'help' to get the list of commands.\n"
	           "Use UP/DOWN arrows to navigate through command history.\n"
			   "Press TAB when typing command name to auto-complete.\n");

	int probe_status = linenoiseProbe();
	if (probe_status) {
		printf("\n"
		               "Your terminal application does not support escape sequences.\n"
		               "Line editing and history features are disabled.\n"
					   "On Windows, try using Putty instead.\n");
		linenoiseSetDumbMode(1);
#if CONFIG_LOG_COLORS
        /* Since the terminal doesn't support escape sequences,
         * don't use color codes in the prompt.
         */
        prompt = "espbox> ";
#endif //CONFIG_LOG_COLORS

	}
	for(;;) {
	       /* Get a line using linenoise.
	         * The line is returned when ENTER is pressed.
	         */
	        char* line = linenoise(prompt);
	        ESP_LOGI(TAG, "1"); //!!!
	        if (line == NULL) { /* Ignore empty lines */
	            continue;
	        }
	        /* Add the command to the history */
	        linenoiseHistoryAdd(line);
	#if CONFIG_STORE_HISTORY
	        /* Save command history to filesystem */
	        linenoiseHistorySave(HISTORY_PATH);
	#endif

	        /* Try to run the command */
	        int ret;
	        esp_err_t err = esp_console_run(line, &ret);
	        if (err == ESP_ERR_NOT_FOUND) {
	            printf("Unrecognized command\n");
	        } else if (err == ESP_ERR_INVALID_ARG) {
	            // command was empty
	        } else if (err == ESP_OK && ret != ESP_OK) {
	            printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(err));
	        } else if (err != ESP_OK) {
	            printf("Internal error: %s\n", esp_err_to_name(err));
	        }
	        /* linenoise allocates line buffer on the heap, so need to free it */
	        linenoiseFree(line);
	}
}

#define CMD_LOG "log"
#define HINT_LOG "off/on"
#define HELP_LOG "on/off log"

static struct {
	struct arg_rex *logon;
	struct arg_rex *logoff;
	struct arg_end *end;
} log_args;

static int cmd_box(int argc, char **argv) {
	int err=arg_parse(argc, argv, (void **)&log_args);
	if (err||log_args.logon->count>0) {
		G_LOG_ON=true;
		return 0;
	}
	if (err||log_args.logoff->count>0) {
		G_LOG_ON=false;
		return 0;
	}
	printf("Log is %s\n", G_LOG_ON?"true":"false");
	return 0;
}

static void register_log_cmd() {
	log_args.logon=arg_rex1(NULL, NULL, "on", NULL, ARG_REX_ICASE, NULL);
	log_args.logoff=arg_rex1(NULL, NULL, "off", NULL, ARG_REX_ICASE, NULL);
	log_args.end=arg_end(0);

	esp_console_cmd_t cmd = {
		.command=CMD_LOG,
		.help=HELP_LOG,
		.hint=HINT_LOG,
		.func=cmd_box
	};

	esp_console_cmd_register(&cmd);
}


esp_err_t console_init() {
	fflush(stdout);
	fsync(fileno(stdout));

	setvbuf(stdin, NULL, _IONBF, 0);
	esp_vfs_dev_uart_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
	esp_vfs_dev_uart_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

	const uart_config_t uart_config = {
			.baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.source_clk = UART_SCLK_REF_TICK
	};

	ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));
	ESP_ERROR_CHECK(uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config));

	esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

	esp_console_config_t console_config = {
			.max_cmdline_args = 16,
			.max_cmdline_length = 256,
#if CONFIG_LOG_COLORS
			.hint_color = atoi(LOG_COLOR_CYAN)
#endif
	};

	ESP_ERROR_CHECK_WITHOUT_ABORT(esp_console_init(&console_config));

	linenoiseSetMultiLine(1);
	linenoiseSetCompletionCallback(&esp_console_get_completion);
	linenoiseSetHintsCallback((linenoiseHintsCallback *) (&esp_console_get_hint));
	linenoiseHistorySetMaxLen(CONSOLE_HIST_LINE_SIZE);
#if CONFIG_STORE_HISTORY
	linenoiseHistoryLoad(HISTORY_PATH);
#endif

	esp_console_register_help_command();
	register_system();
	register_log_cmd();
	tsk_console=xTaskCreateStatic(console_func, "cons", 4096, NULL,
						uxTaskPriorityGet(NULL), stack_console, &buf_console);

    return tsk_console?pdPASS:pdFAIL;
}
