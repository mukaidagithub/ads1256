#ifndef __TEST_SPI__
#define __TEST_SPI__

uint8_t read_reg(uint8_t addr);
void write_reg(uint8_t addr, uint8_t val, uint8_t mask);
void dump_reg(void);
int32_t device_rdata(void);
void device_standby(void);
void device_wakeup(void);
void switch_sensor(uint8_t sensor);
void switch_ch(uint8_t ch);
uint32_t get_gain(void);

void device_preparation(void);
void spi_init(void);
void spi_open(void);
void spi_close(void);


#endif /* __TEST_SPI__0 */
