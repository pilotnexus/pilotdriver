**** CHANGELOG ****
2.0.6 - Improved postinst and prerm scripts
2.0.3 - Forced values
2.0.2 - File permissions fix
2.0.3 - Message length fix for tty enable commands
2.0.5 - Added demo module support
2.0.6 - set init_uart_clock to 48MHz instead of 3MHz to support higher baudrates
2.0.7 - ignore SPI dummy frame from slave
2.0.8 - PLC virt filesystem changes, support for uart_mode 0/1
2.2.1 - copy_to_user in pilot_plc_proc_var_read() introduced
2.2.2 - aio20 module now has afe0_3/afe4_7/afe8_11/afe12_15 to display analog frontend type (0-10V/4-20mA/PT-1000)
2.2.3 - forced values
2.2.4 - fixed variable_count / set when kzalloc done to prevent read thread to access non-initialized variables
2.2.5 - per-variable access-lock. pre-work for multi-consumer variables
