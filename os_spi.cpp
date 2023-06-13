#include "global_includes.h"
#include "driver/spi_master.h"

typedef struct _os_spi_t{
    os_spi_t *spi;
    spi_device_interface_config_t if_cfg;
    spi_bus_config_t bus_cfg;
    spi_host_device_t host; 
    spi_device_handle_t handle;
    int dma_chan;
    uint8_t *tx_buff;
    uint8_t *rx_buff;
}_os_spi_t;

#define SPI_TRANS_MAX_SIZE (3084 * sizeof(uint8_t))

/**
 * @brief our SPI interfaces initialized here
*/
os_spi_t os_spi2 = 
{
    .fd = 2
};
_os_spi_t int_os_spi2
{
    .spi = &os_spi2, 
    .if_cfg = {
        .command_bits = 0,  // 0-16
        .address_bits = 0,  // 0-64
        .dummy_bits = 0,
        .mode = SPI_MODE0,
        .duty_cycle_pos = 128,  // default: 128
        .cs_ena_pretrans = 0,   // only for half-duplex
        .cs_ena_posttrans = 0,
        .clock_speed_hz = SPI_MASTER_FREQ_8M,
        .input_delay_ns = 0,
        .spics_io_num = 15,  // HSPI
        .flags = 0,
        .queue_size = 3,
        .pre_cb = NULL,
        .post_cb = NULL,
    }, 
    .bus_cfg = {
        .mosi_io_num = 23,        // HSPI
        .miso_io_num = 19,        // HSPI
        .sclk_io_num = 18,        // HSPI
        .max_transfer_sz = 4092,  // default: 4092 if DMA enabled, SOC_SPI_MAXIMUM_BUFFER_SIZE if DMA disabled
        .flags = SPICOMMON_BUSFLAG_MASTER,
    }, 
    .host = SPI2_HOST, 
    .dma_chan = 1

};

os_spi_t os_spi3 = {
    .fd = 3
};
_os_spi_t int_os_spi3 = {
    .spi = &os_spi3,
    .if_cfg = {
        .command_bits = 0,  // 0-16
        .address_bits = 0,  // 0-64
        .dummy_bits = 0,
        .mode = SPI_MODE0,
        .duty_cycle_pos = 128,  // default: 128
        .cs_ena_pretrans = 0,   // only for half-duplex
        .cs_ena_posttrans = 0,
        .clock_speed_hz = SPI_MASTER_FREQ_8M,
        .input_delay_ns = 0,
        .spics_io_num = 15,  // HSPI
        .flags = 0,
        .queue_size = 3,
        .pre_cb = NULL,
        .post_cb = NULL,
    }, 
    .bus_cfg = {
        .mosi_io_num = 13,        // HSPI
        .miso_io_num = 12,        // HSPI
        .sclk_io_num = 14,        // HSPI
        .max_transfer_sz = 4092,  // default: 4092 if DMA enabled, SOC_SPI_MAXIMUM_BUFFER_SIZE if DMA disabled
        .flags = SPICOMMON_BUSFLAG_MASTER,
    }, 
    .host = SPI3_HOST, 
    .dma_chan = 2
};

_os_spi_t *map_spi(int fd){
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
static uint8_t* allocate_dma_buffer(const size_t n){
    return (uint8_t*)heap_caps_malloc(n, MALLOC_CAP_DMA);
}

static int _spi_initialize(_os_spi_t *spi){
    spi->bus_cfg.flags |= SPICOMMON_BUSFLAG_MASTER;

    esp_err_t e = spi_bus_initialize(spi->host, &spi->bus_cfg, spi->dma_chan);
    if(e != ESP_OK){
        return OS_RET_INT_ERR;
    }

    e = spi_bus_add_device(spi->host, &spi->if_cfg, &spi->handle);

    if(e != ESP_OK){
        return OS_RET_INT_ERR;
    }

    return OS_RET_OK;
}

static int _spi_deinit(_os_spi_t *spi){
    esp_err_t e = spi_bus_remove_device(spi->handle);

    if(e != ESP_OK){
        return OS_RET_INT_ERR;
    }

    e = spi_bus_free(spi->host);
    if(e != ESP_OK){
        return OS_RET_INT_ERR;
    }

    return OS_RET_OK;
}

int os_spi_begin(os_spi_t *spi){
    if(spi == NULL){
        return OS_RET_NULL_PTR;
    }

    _os_spi_t *_spi = map_spi(spi->fd);
    if(spi == NULL){
        return OS_RET_INVALID_PARAM;
    }

    _spi->rx_buff = allocate_dma_buffer(SPI_TRANS_MAX_SIZE);
    _spi->tx_buff = allocate_dma_buffer(SPI_TRANS_MAX_SIZE);

    if(_spi->rx_buff == NULL | _spi->tx_buff == NULL){
        return OS_RET_LOW_MEM_ERROR;
    }

    // Clear contents of data
    memset(_spi->rx_buff, 0, SPI_TRANS_MAX_SIZE);
    memset(_spi->tx_buff, 0, SPI_TRANS_MAX_SIZE);

    // Call esp32 commands to initialize spi
    return _spi_initialize(_spi);
}

int os_spi_end(os_spi_t *spi){
    if(spi == NULL){
        return OS_RET_NULL_PTR;
    }

    _os_spi_t *_spi = map_spi(spi->fd);
    if(spi == NULL){
        return OS_RET_INVALID_PARAM;
    }

    return OS_RET_OK;
}

int os_spi_setbus(os_spi_t *spi, uint32_t freq_hz){
    if(spi == NULL){
        return OS_RET_NULL_PTR;
    }

    _os_spi_t *_spi = map_spi(spi->fd);
    if(spi == NULL){
        return OS_RET_INVALID_PARAM;
    }

    return OS_RET_OK;
}

int os_spi_transfer(os_spi_t *spi, uint8_t *rx, uint8_t *tx, size_t size){
    if(spi == NULL){
        return OS_RET_NULL_PTR;
    }

    _os_spi_t *_spi = map_spi(spi->fd);
    if(spi == NULL){
        return OS_RET_INVALID_PARAM;
    }

    spi_transaction_t trans; 

    trans.flags = 0;
    trans.flags |= SPI_TRANS_VARIABLE_CMD; 
    trans.flags |= SPI_TRANS_VARIABLE_ADDR;
    trans.flags |= SPI_TRANS_VARIABLE_DUMMY;
    
    trans.addr = 0;
    trans.cmd = 0;
    trans.length = 8 * size;
    trans.rxlength = 0;
    trans.user = NULL;
    trans.tx_buffer = _spi->tx_buff;
    trans.rx_buffer = _spi->rx_buff;
    esp_err_t e = spi_device_transmit(_spi->handle, &trans);

    if(e != ESP_OK){
        return OS_RET_INT_ERR;
    }

    return OS_RET_OK;
}

int os_spi_send(os_spi_t *spi, uint8_t *buf, size_t size){
    if(spi == NULL){
        return OS_RET_NULL_PTR;
    }

    _os_spi_t *_spi = map_spi(spi->fd);
    if(spi == NULL){
        return OS_RET_INVALID_PARAM;
    }
    return OS_RET_OK;
}

int os_spi_recieve(os_spi_t *spi, uint8_t *buf, size_t size){
    if(spi == NULL){
        return OS_RET_NULL_PTR;
    }

    _os_spi_t *_spi = map_spi(spi->fd);
    if(spi == NULL){
        return OS_RET_INVALID_PARAM;
    }

    return OS_RET_OK;
}