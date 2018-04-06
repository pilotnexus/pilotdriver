
void flash_read_id(module_slot_t slot, char* buffer);
void flash_power_up(module_slot_t slot);
void flash_power_down(module_slot_t slot);
void flash_write_enable(module_slot_t slot);
void flash_bulk_erase(module_slot_t slot);
void flash_64kB_sector_erase(module_slot_t slot, int addr);
bool flash_prog(module_slot_t slot, int addr, char *data, int n);
bool flash_read(module_slot_t slot, int addr, char *data, int n);
void flash_wait(module_slot_t slot);
bool flash_busy(module_slot_t slot);
bool is_flash_write_enabled_and_not_busy(module_slot_t slot);