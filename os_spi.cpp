#include "global_includes.h"
#include "driver/spi_master.h"
#include <esp_heap_caps.h>

#define SPI_DEBUGGING
#ifdef SPI_DEBUGGING
#define SPI_DEBUG(e, ...)                  \
    os_printf("[SPI MATRIX DEBUGGING]: "); \
    os_printf(e);                          \
    os_printf("\n")
#else
#define SPI_DEBUG(e, ...) (void)(e)
#endif

/**
 * @brief Internal SPI initialization structures for the esp32 platform
 */
typedef struct _os_spi_t
{
    spi_bus_config_t bus_cfg;
    spi_host_device_t host;
    spi_dma_chan_t dma_chan;
} _os_spi_t;

#define SPI_TRANS_MAX_SIZE (1024 * sizeof(uint16_t))

/**
 * @brief SPI2 and SPI3 configuration, setting based off default GPIOs
 */
_os_spi_t int_os_spi2 = {
    .bus_cfg = {
        .mosi_io_num = 35, // HSPI

        .miso_io_num = 37, // HSPI
        .sclk_io_num = 36, // HSPI
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,

        .flags = SPICOMMON_BUSFLAG_MASTER,
    },
    .host = SPI2_HOST,
    .dma_chan = SPI_DMA_CH_AUTO};
_os_spi_t int_os_spi3 = {
    .bus_cfg = {
        .mosi_io_num = 13, // HSPI
        .miso_io_num = 12, // HSPI
        .sclk_io_num = 14, // HSPI
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
        .flags = SPICOMMON_BUSFLAG_MASTER,
    },
    .host = SPI3_HOST,
    .dma_chan = SPI_DMA_CH_AUTO};

/**
 * @brief based off fd, we can figure out which spi we are refering to
 */
_os_spi_t *map_spi(int fd)
{
    switch (fd)
    {
    case 2:
        return &int_os_spi2;
        break;

    case 3:
        return &int_os_spi3;

    default:
        return NULL;
    }
}

typedef struct _os_device_t
{
    uint8_t *rx_buf;
    uint8_t *tx_buf;
    spi_device_interface_config_t devcfg;
    spi_device_handle_t handle;
} _os_device_t;

static uint8_t *allocate_dma_buffer(size_t n)
{
    return (uint8_t *)heap_caps_malloc(n, MALLOC_CAP_DMA);
}

int os_spi_initialize(os_spi_t *spi, int fd, os_spi_gpio_t *gpio)
{
    if (spi == NULL)
    {
        SPI_DEBUG("SPI NULL POINTER");
        return OS_RET_NULL_PTR;
    }

    // Set map for internal spi structures
    spi->fd = fd;
    _os_spi_t *int_spi = map_spi(spi->fd);

    if (int_spi == NULL)
    {
        return OS_RET_INVALID_PARAM;
    }

    if (gpio)
    {
        spi->gpio_man = *gpio;
        int_spi->bus_cfg.mosi_io_num = gpio->mosi;
        int_spi->bus_cfg.miso_io_num = gpio->miso;
        int_spi->bus_cfg.sclk_io_num = gpio->clk;
    }

    esp_err_t e = spi_bus_initialize(int_spi->host, &int_spi->bus_cfg, int_spi->dma_chan);
    int err = esp_to_os(e);
    if (e != ESP_OK)
    {
        SPI_DEBUG("ESPIDF FAILED TO INITIALIZE");
        Serial.printf("Failed to initialize spi bus : %d\n", err);
        return OS_RET_INT_ERR;
    }

    return OS_RET_OK;
}

int os_spi_couple_device(os_device_init_params init_params, os_device_t *device)
{
    if (device == NULL)
    {
        Serial.println("Failed to find device");
        SPI_DEBUG("FAILED TO FIND DEVICE");
        return OS_RET_NULL_PTR;
    }

    if (init_params.bus == NULL)
    {
        Serial.println("Failed to find bus");
        SPI_DEBUG("FAILED TO FIND SPI BUS");
        return OS_RET_NULL_PTR;
    }

    device->chip_select = init_params.cs_gpio;
    device->dma_buf_size = init_params.dma_buf_size;
    device->clk = init_params.clk;
    device->bus = init_params.bus;
    device->spi_mode = init_params.spi_mode;

    device->device = malloc(sizeof(_os_device_t));
    _os_device_t *spi_device = (_os_device_t *)device->device;

    spi_device->rx_buf = allocate_dma_buffer(device->dma_buf_size);
    spi_device->tx_buf = allocate_dma_buffer(device->dma_buf_size);

    memset(spi_device->rx_buf, 0, SPI_TRANS_MAX_SIZE);
    memset(spi_device->rx_buf, 0, SPI_TRANS_MAX_SIZE);
    spi_device->devcfg = {
        .mode = device->spi_mode, // SPI mode 0
        .clock_speed_hz = device->clk,
        .spics_io_num = device->chip_select, // CS pin
        .queue_size = 2,                     // We want to be able to queue 7 transactions at a time
        .pre_cb = NULL,                      // Specify pre-transfer callback to handle D/C line
    };
    if (device->bus == NULL)
    {
        SPI_DEBUG("FAILED TO FIND MAP TO SPI BUS FRONTEND");
    }
    _os_spi_t *int_bus = map_spi(device->bus->fd);
    if (int_bus == NULL)
    {
        SPI_DEBUG("FAILED TO FIND MAP TO CORRECT INTERFACE %d", device->bus->fd);
        return OS_RET_INVALID_PARAM;
    }

    esp_err_t e = spi_bus_add_device(int_bus->host, &spi_device->devcfg, &spi_device->handle);
    int err = esp_to_os(e);
    if (err != OS_RET_OK)
    {
        Serial.printf("Failed to couple SPI device to interface: %d\n", err);
        return err;
    }
    return OS_RET_OK;
}

int os_spi_decouple_device(os_device_t *device)
{
    if (device == NULL | device->bus == NULL | device->device == NULL)
    {
        return OS_RET_INVALID_PARAM;
    }

    _os_device_t *spi_device = (_os_device_t *)device->device;
    // Free DMA buffers
    heap_caps_free(spi_device->rx_buf);
    heap_caps_free(spi_device->tx_buf);

    // Remove device from spi bus
    spi_bus_remove_device(spi_device->handle);

    // Free device resources
    free(device->device);
    // flag set
    device->device = NULL;
    device->bus = NULL;
    return OS_RET_OK;
}

int os_spi_end(os_spi_t *spi)
{
    if (spi == NULL)
    {
        return OS_RET_NULL_PTR;
    }

    _os_spi_t *_spi = map_spi(spi->fd);
    if (_spi == NULL)
    {
        return OS_RET_INVALID_PARAM;
    }

    esp_err_t e = spi_bus_free(_spi->host);
    int err = esp_to_os(e);
    if (err == OS_RET_OK)
    {
        spi->fd = 0;
    }
    return err;
}

int os_spi_transfer(os_device_t *device, uint8_t *rx, uint8_t *tx, size_t size)
{
    if (device == NULL | device->bus == NULL | device->device == NULL)
    {
        return OS_RET_NULL_PTR;
    }

    _os_device_t *spi_device = (_os_device_t *)device->device;

    // copy data from our buffer into the tx buffer
    memcpy(spi_device->tx_buf, tx, size);

    // Setup transaction details
    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(transaction));
    transaction.flags = 0;
    transaction.rx_buffer = spi_device->rx_buf;
    transaction.tx_buffer = spi_device->tx_buf;
    transaction.length = size;

    spi_device_acquire_bus(spi_device->handle, portMAX_DELAY);
    esp_err_t e = spi_device_transmit(spi_device->handle, &transaction);
    spi_device_release_bus(spi_device->handle);

    int err = esp_to_os(e);
    if (err != OS_RET_OK)
    {
        return err;
    }

    // Copy contents back into the RX buffer
    memcpy(rx, spi_device->rx_buf, size);
    return err;
}

int os_spi_send(os_device_t *device, uint8_t *buf, size_t size)
{
    if (device == NULL | device->bus == NULL | device->device == NULL)
    {
        return OS_RET_NULL_PTR;
    }

    _os_device_t *spi_device = (_os_device_t *)device->device;

    Serial.println("SPI send matrix");
    // copy data from our buffer into the tx buffer
    memcpy(spi_device->tx_buf, buf, size);

    // Setup transaction details
    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(transaction));
    transaction.flags = 0;
    transaction.rx_buffer = spi_device->rx_buf;
    transaction.tx_buffer = spi_device->tx_buf;
    transaction.length = size * 8;

    spi_device_acquire_bus(spi_device->handle, portMAX_DELAY);
    esp_err_t e = spi_device_transmit(spi_device->handle, &transaction);
    spi_device_release_bus(spi_device->handle);

    int err = esp_to_os(e);
    if (err != OS_RET_OK)
    {
        return err;
    }
    return err;
}

int os_spi_recieve(os_device_t *device, uint8_t *buf, size_t size)
{
    if (device == NULL | device->bus == NULL | device->device == NULL)
    {
        return OS_RET_NULL_PTR;
    }

    _os_device_t *spi_device = (_os_device_t *)device->device;

    if (size > device->dma_buf_size)
    {
        return OS_RET_NO_MORE_RESOURCES;
    }
    // Setup transaction details
    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(transaction));
    transaction.flags = 0;
    transaction.rx_buffer = spi_device->rx_buf;
    transaction.tx_buffer = spi_device->tx_buf;
    transaction.length = size;

    spi_device_acquire_bus(spi_device->handle, portMAX_DELAY);
    esp_err_t e = spi_device_transmit(spi_device->handle, &transaction);
    spi_device_release_bus(spi_device->handle);

    int err = esp_to_os(e);
    if (err != OS_RET_OK)
    {
        return err;
    }

    // Copy contents back into the RX buffer
    memcpy(buf, spi_device->rx_buf, size);
    return err;
}