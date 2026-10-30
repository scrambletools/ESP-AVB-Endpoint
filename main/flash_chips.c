/*
 * Flash chip drivers for the P4 boards (CONFIG_SPI_FLASH_OVERRIDE_CHIP_DRIVER_LIST).
 *
 * The boards carry a GigaDevice GD25Q256 (JEDEC C84019). ESP-IDF's GD driver
 * grants flash auto-suspend only to the GD25Q32/64/128 IDs it lists, so with
 * CONFIG_SPI_FLASH_AUTO_SUSPEND the boards failed flash init at boot. The
 * GD25Q256 has the same suspend scheme as those parts: suspend 75h, resume
 * 7Ah, SUS1 at S15 and SUS2 at S10 of status register 2, which is what the GD
 * driver's suspend setup (mask 0x84 on SR2) expects. This list adds a copy of
 * the GD driver that matches only that ID and grants suspend; every other chip
 * keeps IDF's own driver, in IDF's own order.
 *
 * Auto-suspend keeps the caches enabled during an erase or program, so an NVS
 * write no longer stalls everything that runs from flash: a page erase used to
 * hold the cache off for 50 ms, which overflowed the EMAC receive ring.
 */
#include "sdkconfig.h"

#if CONFIG_SPI_FLASH_OVERRIDE_CHIP_DRIVER_LIST

#include "esp_flash_chips/spi_flash_chip_driver.h"
#include "esp_flash_chips/spi_flash_chip_gd.h"
#include "esp_flash_chips/spi_flash_chip_generic.h"
#include "esp_flash_chips/spi_flash_chip_winbond.h"
#ifdef CONFIG_SPI_FLASH_SUPPORT_ISSI_CHIP
#include "esp_flash_chips/spi_flash_chip_issi.h"
#endif
#ifdef CONFIG_SPI_FLASH_SUPPORT_MXIC_CHIP
#include "esp_flash_chips/spi_flash_chip_mxic.h"
#endif
#ifdef CONFIG_SPI_FLASH_SUPPORT_BOYA_CHIP
#include "esp_flash_chips/spi_flash_chip_boya.h"
#endif
#ifdef CONFIG_SPI_FLASH_SUPPORT_TH_CHIP
#include "esp_flash_chips/spi_flash_chip_th.h"
#endif

/* Defined in IDF's spi_flash_chip_gd.c and spi_flash_chip_winbond.c, not in
 * their public headers. */
esp_err_t spi_flash_chip_gd_detect_size(esp_flash_t *chip, uint32_t *size);
spi_flash_caps_t spi_flash_chip_gd_get_caps(esp_flash_t *chip);
esp_err_t spi_flash_chip_gd_suspend_cmd_conf(esp_flash_t *chip);
esp_err_t spi_flash_chip_winbond_read(esp_flash_t *chip, void *buffer,
                                      uint32_t address, uint32_t length);
esp_err_t spi_flash_chip_winbond_page_program(esp_flash_t *chip,
                                              const void *buffer,
                                              uint32_t address,
                                              uint32_t length);
esp_err_t spi_flash_chip_winbond_erase_sector(esp_flash_t *chip,
                                              uint32_t start_address);
esp_err_t spi_flash_chip_winbond_erase_block(esp_flash_t *chip,
                                             uint32_t start_address);

#define GD25Q256_ID 0xC84019

static esp_err_t gd25q256_probe(esp_flash_t *chip, uint32_t flash_id) {
  (void)chip;
  return flash_id == GD25Q256_ID ? ESP_OK : ESP_ERR_NOT_FOUND;
}

static spi_flash_caps_t gd25q256_get_caps(esp_flash_t *chip) {
  spi_flash_caps_t caps = spi_flash_chip_gd_get_caps(chip);
#if CONFIG_SPI_FLASH_AUTO_SUSPEND
  caps |= SPI_FLASH_CHIP_CAP_SUSPEND;
#endif
  return caps;
}

static const char gd25q256_name[] = "gd25q256";

/* IDF's esp_flash_chip_gd with the probe and capabilities above; GD reads,
 * programs and erases with the Winbond functions. */
static const spi_flash_chip_t esp_flash_chip_gd25q256 = {
    .name = gd25q256_name,
    .timeout = &spi_flash_chip_generic_timeout,
    .probe = gd25q256_probe,
    .reset = spi_flash_chip_generic_reset,
    .detect_size = spi_flash_chip_gd_detect_size,
    .erase_chip = spi_flash_chip_generic_erase_chip,
    .erase_sector = spi_flash_chip_winbond_erase_sector,
    .erase_block = spi_flash_chip_winbond_erase_block,
    .sector_size = 4 * 1024,
    .block_erase_size = 64 * 1024,

    .get_chip_write_protect = spi_flash_chip_generic_get_write_protect,
    .set_chip_write_protect = spi_flash_chip_generic_set_write_protect,

    .num_protectable_regions = 0,
    .protectable_regions = NULL,
    .get_protected_regions = NULL,
    .set_protected_regions = NULL,

    .read = spi_flash_chip_winbond_read,
    .write = spi_flash_chip_generic_write,
    .program_page = spi_flash_chip_winbond_page_program,
    .page_size = 256,
    .write_encrypted = spi_flash_chip_generic_write_encrypted,

    .wait_idle = spi_flash_chip_generic_wait_idle,
    .set_io_mode = spi_flash_chip_gd_set_io_mode,
    .get_io_mode = spi_flash_chip_gd_get_io_mode,

    .read_reg = spi_flash_chip_generic_read_reg,
    .yield = spi_flash_chip_generic_yield,
    .sus_setup = spi_flash_chip_gd_suspend_cmd_conf,
    .read_unique_id = spi_flash_chip_generic_read_unique_id,
    .get_chip_caps = gd25q256_get_caps,
    .config_host_io_mode = spi_flash_chip_generic_config_host_io_mode,
};

/* IDF's default list (spi_flash_chip_drivers.c) with the GD25Q256 first. */
const spi_flash_chip_t *default_registered_chips[] = {
    &esp_flash_chip_gd25q256,
#ifdef CONFIG_SPI_FLASH_SUPPORT_ISSI_CHIP
    &esp_flash_chip_issi,
#endif
#ifdef CONFIG_SPI_FLASH_SUPPORT_GD_CHIP
    &esp_flash_chip_gd,
#endif
#ifdef CONFIG_SPI_FLASH_SUPPORT_MXIC_CHIP
    &esp_flash_chip_mxic,
#endif
#ifdef CONFIG_SPI_FLASH_SUPPORT_WINBOND_CHIP
    &esp_flash_chip_winbond,
#endif
#ifdef CONFIG_SPI_FLASH_SUPPORT_BOYA_CHIP
    &esp_flash_chip_boya,
#endif
#ifdef CONFIG_SPI_FLASH_SUPPORT_TH_CHIP
    &esp_flash_chip_th,
#endif
    &esp_flash_chip_generic,
    NULL,
};

#endif /* CONFIG_SPI_FLASH_OVERRIDE_CHIP_DRIVER_LIST */
