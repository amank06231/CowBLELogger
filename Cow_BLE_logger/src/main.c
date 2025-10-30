/* ========================================================
 * Accelerometer Data Logger with Flash + Extended BLE Advertisements of 236 bytes
 * Target: nRF52832 with Zephyr RTOS
 * ======================================================== */

#include <zephyr/zephyr.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/controller.h>
#include <zephyr/pm/pm.h>
#include <hal/nrf_gpio.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/* ============================
 * Flash Layout (2MB & 8MB Supported)
 * ============================ */
#define FLASH_START_ADDR 0x00000000            // Flash start address
#define FLASH_TOTAL_SIZE_8MB (8 * 1024 * 1024) // 8MB flash
#define FLASH_TOTAL_SIZE_2MB (2 * 1024 * 1024) // 2MB flash
#define SPI_FLASH_SECTOR_SIZE 4096             // Sector size = 4KB

#if defined(CONFIG_FLASH_SIZE_8MB)
#define FLASH_TOTAL_SIZE FLASH_TOTAL_SIZE_8MB
#else
#define FLASH_TOTAL_SIZE FLASH_TOTAL_SIZE_2MB
#endif

#define FLASH_END_ADDR (FLASH_START_ADDR + FLASH_TOTAL_SIZE)                               // Flash end
#define FLASH_STATE_SIZE 0x1000                                                            // 4KB reserved for state
#define FLASH_RESET_BACKUP_SIZE 0x1000                                                     // 4KB reserved for reset backup
#define FLASH_STORAGE_SIZE (FLASH_TOTAL_SIZE - FLASH_STATE_SIZE - FLASH_RESET_BACKUP_SIZE) // Usable area
#define FLASH_STORAGE_ADDR FLASH_START_ADDR                                                // Data starts at 0x0
#define FLASH_STATE_ADDR (FLASH_STORAGE_ADDR + FLASH_STORAGE_SIZE)                         // State offset
#define FLASH_RESET_BACKUP_ADDR (FLASH_STATE_ADDR + FLASH_STATE_SIZE)                      // Reset backup offset

static int flashLimit = (int)FLASH_STORAGE_SIZE;
static int save_location = (int)FLASH_STATE_ADDR;
static const int STATE_ADDR = (int)FLASH_STATE_ADDR;
static const int STATE_SIZE = (int)FLASH_STATE_SIZE;

/* ============================
 * Global Variables
 * ============================ */
K_MUTEX_DEFINE(testMutex);

/* Flash / State variables */
static int writeIndex = 0;
static int readIndex = 0;
static int package_count = 0;
static int file_count = 0;
static int write_storage[2] = {0, 0};
/* TX Buffer Config */
#define TX_HEADER_SIZE 2
#define TX_PAYLOAD_SIZE 231
#define TX_FOOTER_SIZE 3
#define TX_PACKET_SIZE 256
#define TX_BUFFER_SIZE TX_PACKET_SIZE                                     // Keep for compatibility
#define TX_VALID_SIZE (TX_HEADER_SIZE + TX_PAYLOAD_SIZE + TX_FOOTER_SIZE) // Header + Payload + Footer = 236 bytes
// #define TX_PACKET_SIZE (TX_HEADER_SIZE + TX_PAYLOAD_SIZE + TX_FOOTER_SIZE) // Total = 256 bytes

static uint8_t tx_buffer[TX_BUFFER_SIZE] = {0}; // 0..255 → 256 bytes
uint8_t dummy_buf[TX_VALID_SIZE] = {0};         // All zeros
static bool dummy_sent = false;                 // <-- Add this as a static/global variable
static uint8_t mfg_data[TX_VALID_SIZE] = {0};   // Match size exactly to avoid overflow
static uint8_t tx_index = 0;
static uint8_t packet_id = 1;

/* BLE / Advertising Variables */
static struct bt_le_ext_adv *adv;
static struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
    BT_LE_ADV_OPT_NONE | BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_USE_IDENTITY | BT_LE_ADV_OPT_NO_2M,
    0x30, 0x30, NULL);

static struct bt_le_ext_adv_start_param ext_adv_param = BT_LE_EXT_ADV_START_PARAM_INIT(0, 1);
static const struct bt_data ap[] = {
    BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)), // 236 bytes of data
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};
static const struct bt_data ad[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* Button + LED Config */
#define SW0_NODE DT_ALIAS(sw0)
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

/* Sensor Config */
static const struct device *sensor = DEVICE_DT_GET_ANY(st_lis2dh);
#define nodeId CONFIG_NODE_ID
#define sensorAddress CONFIG_SENSOR_ADDRESS

/* Work & Timer */
static struct k_work adver_work;
static struct k_timer button_timer;
static struct k_work acc_data_work;
static struct k_timer acc_data_timer;
static volatile bool task2_active = false;

/* ============================
 * Utility Functions
 * ============================ */
static inline double roundOff(float value)
{
    return (value > 0) ? (value + 0.5f) : (value - 0.5f);
}

struct sys_state
{
    int writeIndex;
    int file_count;
    int readIndex;
} state;

/* ============================
 * Flash State Save/Restore
 * ============================ */
void save_state(const struct device *flashDev)
{
    state.writeIndex = writeIndex;
    state.file_count = file_count;
    state.readIndex = readIndex;

    if (flash_erase(flashDev, STATE_ADDR, STATE_SIZE))
    {
        printk("Flash erase failed\n");
        return;
    }
    if (flash_write(flashDev, STATE_ADDR, &state, sizeof(state)))
    {
        printk("Flash write failed\n");
        return;
    }
    printk("State saved: writeIndex=%d file_count=%d readIndex=%d\n", writeIndex, file_count, readIndex);
}

void restore_state(const struct device *flashDev)
{
    struct sys_state tmp;
    if (flash_read(flashDev, STATE_ADDR, &tmp, sizeof(tmp)))
    {
        printk("Flash read failed\n");
        return;
    }
    if (tmp.writeIndex == -1 || tmp.writeIndex > FLASH_END_ADDR)
    {
        printk("No valid state found, starting fresh\n");
        writeIndex = file_count = readIndex = 0;
    }
    else
    {
        writeIndex = tmp.writeIndex;
        file_count = tmp.file_count;
        readIndex = tmp.readIndex;
        printk("Restored -> writeIndex=%d, file_count=%d, readIndex=%d\n", writeIndex, file_count, readIndex);
    }
}

/* ============================
 * BLE Setup & Transmission
 * ============================ */
void set_random_static_address(void)
{
    bt_addr_le_t addr;
    if (bt_addr_le_from_str(sensorAddress, "random", &addr))
    {
        printk("Invalid BT address\n");
    }
    if (bt_id_create(&addr, NULL) < 0)
    {
        printk("Creating new ID failed\n");
    }
    else
    {
        printk("Created new address\n");
    }
}

void adv_param_init(void)
{
    if (bt_le_ext_adv_create(&adv_param, NULL, &adv))
    {
        printk("Failed to create advertising set\n");
    }
    else
    {
        printk("Created extended advertising set\n");
    }
}

void transmit_name()
{
    if (bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0))
    {
        printk("Failed to set adv data\n");
        return;
    }
    printk("\nStart Extended Advertising...\n");
    if (bt_le_ext_adv_start(adv, &ext_adv_param))
    {
        printk("Failed to start advertising\n");
        return;
    }
    k_msleep(100);
    bt_le_ext_adv_stop(adv);
    printk("Stopped advertising.\n\n");
}
void transmit_data()
{
    if (bt_le_ext_adv_set_data(adv, ap, ARRAY_SIZE(ap), NULL, 0))
    {
        printk("Failed to set adv data\n");
        return;
    }
    printk("\nStart Extended Advertising...\n");
    if (bt_le_ext_adv_start(adv, &ext_adv_param))
    {
        printk("Failed to start advertising\n");
        return;
    }
    k_msleep(100);
    bt_le_ext_adv_stop(adv);
    printk("Stopped advertising.\n\n");
}

/* ============================
 * Flash Write Logic
 * ============================ */
static int write_to_flash(uint8_t *data, size_t len)
{
    if (task2_active)
    {
        printk("Task2 active -> skipping flash write\n");
        bt_le_ext_adv_stop(adv);
        return 1;
    }
    const struct device *flashDev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
    if (writeIndex % SPI_FLASH_SECTOR_SIZE == 0)
    {
        if (flash_erase(flashDev, writeIndex, SPI_FLASH_SECTOR_SIZE))
        {
            printk("Flash erase failed\n");
            return -1;
        }
        printk("Erased sector at 0x%08X\n", writeIndex);
    }
    if (flash_write(flashDev, writeIndex, data, len))
    {
        printk("Flash write failed\n");
        return -1;
    }
    printk("Wrote %d bytes to flash at 0x%08X\n", len, writeIndex);
    writeIndex = (writeIndex + len) % flashLimit;
    if (writeIndex % TX_PAYLOAD_SIZE == 0)
        file_count++;
    save_state(flashDev);
    return 0;
}

/* ============================
 * Accelerometer Fetch + Advertise
 * ============================ */
void fetch_acc_data(void)
{
    if (task2_active)
    {
        printk("Skipping advertising & flash (Task2 active)\n");
        bt_le_ext_adv_stop(adv);
        return;
    }

    if (!device_is_ready(sensor))
    {
        printk("Sensor not ready\n");
        return;
    }

    struct sensor_value accel[3];
    if (sensor_sample_fetch(sensor) == 0 &&
        sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_XYZ, accel) == 0)
    {
        /* Convert accelerometer values to scaled int8_t */
        int8_t xLsb = (int8_t)(roundOff(accel[0].val1 * 6.4 + accel[0].val2 * 0.0000064));
        int8_t yLsb = (int8_t)(roundOff(accel[1].val1 * 6.4 + accel[1].val2 * 0.0000064));
        int8_t zLsb = (int8_t)(roundOff(accel[2].val1 * 6.4 + accel[2].val2 * 0.0000064));

        /* If this is the first sample in a new packet, clear buffer & set header */
        if (tx_index == 0)
        {
            memset(tx_buffer, 0x00, TX_PACKET_SIZE); // Clear full buffer
            tx_buffer[0] = 0x59;                     // Header byte 1
            tx_buffer[1] = 0x00;                     // Header byte 2
        }

        /* Append 3-axis data into buffer */
        tx_buffer[TX_HEADER_SIZE + tx_index++] = xLsb;
        tx_buffer[TX_HEADER_SIZE + tx_index++] = yLsb;
        tx_buffer[TX_HEADER_SIZE + tx_index++] = zLsb;

        printk("X=%d Y=%d Z=%d (tx_index=%d)\n", xLsb, yLsb, zLsb, tx_index); /* printing x , y , z values */

        /* Once we reach 231 bytes of data, finalize and write */
        if (tx_index >= 231)
        {
            tx_buffer[TX_HEADER_SIZE + TX_PAYLOAD_SIZE + 0] = packet_id++; // Packet ID
            tx_buffer[TX_HEADER_SIZE + TX_PAYLOAD_SIZE + 1] = 0x00;        // Reserved
            tx_buffer[TX_HEADER_SIZE + TX_PAYLOAD_SIZE + 2] = 0xFF;        // Footer

            write_to_flash(tx_buffer, TX_PACKET_SIZE); // Write full 256 bytes

            tx_index = 0; // Reset for next packet
        }
    }
}

/* ============================
 * Advertisement Replay Task
 * ============================ */
void adver(void)
{
    while (1)
    {
        int val = gpio_pin_get_dt(&button);
        const struct device *flashDev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
        if (!device_is_ready(flashDev))
            return;

        static int64_t last_tx_time = 0; // Keeps track of last transmission time
        int64_t now = k_uptime_get();
        bool send_name = false;

        // Decide if we should send name (dummy packets)
        if ((now - last_tx_time) > 5000) // If more than 5 sec gap
        {
            send_name = true;
        }
        /* Long press -> reset flash */
        time_t reset_time = k_uptime_get();
        while (val != 1)
        {
            if (k_uptime_get() - reset_time > 5000)
            {
                writeIndex = readIndex = file_count = package_count = packet_id = tx_index = 0;
                write_storage[0] = writeIndex;
                write_storage[1] = file_count;

                k_mutex_lock(&testMutex, K_FOREVER);
                gpio_pin_set_dt(&led1, 1);
                flash_erase(flashDev, save_location, SPI_FLASH_SECTOR_SIZE);
                flash_write(flashDev, save_location, write_storage, sizeof(write_storage));
                gpio_pin_set_dt(&led1, 0);
                k_mutex_unlock(&testMutex);
                printk("Data reset done\n");
                k_msleep(100);
                break;
            }
            val = gpio_pin_get_dt(&button);
        }

        if (val == 1)
        {
            task2_active = true;
            uint32_t pending = (writeIndex >= readIndex) ? (writeIndex - readIndex) : (flashLimit - readIndex + writeIndex);

            if (pending == 0)
            {
                printk("No data to send, starting new fetch\n");
                task2_active = false;
                k_work_submit(&acc_data_work);
                return;
            }

            if (readIndex >= flashLimit)
                readIndex = 0;

            while (pending >= TX_PACKET_SIZE)
            {
                uint32_t chunk = (pending >= TX_PACKET_SIZE) ? TX_PACKET_SIZE : pending;

                /* Read only header+payload+footer from flash */
                if (flash_read(flashDev, readIndex, tx_buffer, TX_VALID_SIZE))
                    break;

                /* ---- Send dummy/name packets only if required ---- */
                if (send_name)
                {
                    for (int pkt = 0; pkt < 20; pkt++)
                    {
                        transmit_name();
                    }
                    send_name = false; // Done once
                }

                /* -------- 2) Now Transmit Actual Data -------- */
                memcpy(mfg_data, tx_buffer, TX_VALID_SIZE);

                printk("Read %d bytes from flash [0x%08X → 0x%08X]\n",
                       chunk, (unsigned int)readIndex, (unsigned int)(readIndex + chunk - 1));
                for (int i = 0; i < TX_VALID_SIZE; i++)
                {
                    printk("%d ", mfg_data[i]);
                    if ((i + 1) % 16 == 0)
                        printk("\n");
                }
                printk("\n");
                gpio_pin_set_dt(&led0, 1);
                transmit_data();
                gpio_pin_set_dt(&led0, 0);

                /* Move to next packet in flash */
                readIndex = (readIndex + chunk) % flashLimit;
                pending -= chunk;
                save_state(flashDev);

                last_tx_time = now; // Update timestamp after successful transmission
            }
            task2_active = false;
        }
        k_msleep(500);
        break;
    }
}

// void adver(void)
// {
//     while (1)
//     {
//         int val = gpio_pin_get_dt(&button);
//         const struct device *flashDev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
//         if (!device_is_ready(flashDev))
//             return;

//         /* Long press -> reset flash */
//         time_t reset_time = k_uptime_get();
//         while (val != 1)
//         {
//             if (k_uptime_get() - reset_time > 5000)
//             {
//                 writeIndex = readIndex = file_count = package_count = packet_id = tx_index = 0;
//                 write_storage[0] = writeIndex;
//                 write_storage[1] = file_count;

//                 k_mutex_lock(&testMutex, K_FOREVER);
//                 gpio_pin_set_dt(&led1, 1);
//                 flash_erase(flashDev, save_location, SPI_FLASH_SECTOR_SIZE);
//                 flash_write(flashDev, save_location, write_storage, sizeof(write_storage));
//                 gpio_pin_set_dt(&led1, 0);
//                 k_mutex_unlock(&testMutex);
//                 printk("Data reset done\n");
//                 k_msleep(100);
//                 break;
//             }
//             val = gpio_pin_get_dt(&button);
//         }

//         if (val == 1)
//         {
//             task2_active = true;
//             uint32_t pending = (writeIndex >= readIndex) ? (writeIndex - readIndex) : (flashLimit - readIndex + writeIndex);

//             if (pending == 0)
//             {
//                 printk("No data to send, starting new fetch\n");
//                 task2_active = false;
//                 k_work_submit(&acc_data_work);
//                 return;
//             }

//             if (readIndex >= flashLimit)
//                 readIndex = 0;

//             while (pending >= TX_BUFFER_SIZE)
//             {
//                 uint32_t chunk = (pending >= TX_PACKET_SIZE) ? TX_PACKET_SIZE : pending;
//                 if (flash_read(flashDev, readIndex, tx_buffer, TX_VALID_SIZE))

//                     break;
//                 memcpy(mfg_data, tx_buffer, TX_VALID_SIZE);

//                 /* printing buffer data*/
//                 printk("Read %d bytes from flash [0x%08X → 0x%08X]\n",
//                        chunk, (unsigned int)readIndex, (unsigned int)(readIndex + chunk - 1));
// for (int i = 0; i < TX_VALID_SIZE; i++)
// {
//     printk("%02X ", tx_buffer[i]);
//     if ((i + 1) % 16 == 0)
//         printk("\n");
// }
// printk("\n");

//                 gpio_pin_set_dt(&led0, 1);
//                 transmit_data();
//                 gpio_pin_set_dt(&led0, 0);

//                 readIndex = (readIndex + chunk) % flashLimit;
//                 pending -= chunk;
//                 save_state(flashDev);
//             }
//             task2_active = false;
//         }
//         k_msleep(500);
//         break;
//     }
// }

/* ============================
 * Button Timer Handler
 * ============================ */
static void button_timer_handler(struct k_timer *dummy)
{
    if (gpio_pin_get_dt(&button) == 0)
    {
        k_work_submit(&adver_work);
    }
}
/* --- Timer Handler: submit accelerometer read work --- */
static void acc_timer_handler(struct k_timer *timer_id)
{
    k_work_submit(&acc_data_work);
}

/* ============================
 * Initialization & Main
 * ============================ */
void sensor_init(void)
{
    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&led0, 0);
    gpio_pin_set_dt(&led1, 0);

    if (!gpio_is_ready_dt(&button))
    {
        printk("Button not ready\n");
        return;
    }
    gpio_pin_configure_dt(&button, GPIO_INPUT);
    k_msleep(4000);
}

int main(void)
{
    sensor_init();
    printk("Starting Data Logger Node\n");

    set_random_static_address();
    int err = bt_enable(NULL); // Enable Bluetooth stack initialization
    if (err)                   // Check if there was an error in initializing the Bluetooth stack
    {
        printk("Bluetooth init failed (err %d)\n", err); // Print an error message indicating the failure to initialize Bluetooth
        return;
    }
    adv_param_init();
    k_msleep(100);

    const struct device *flashDev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
    if (!device_is_ready(flashDev))
        return -1;
    restore_state(flashDev);

    k_work_init(&adver_work, (k_work_handler_t)adver);
    k_timer_init(&button_timer, button_timer_handler, NULL);
    k_timer_start(&button_timer, K_MSEC(50), K_MSEC(50));

    /* --- Accelerometer data timer: fires every 500ms --- */
    k_work_init(&acc_data_work, (k_work_handler_t)fetch_acc_data);
    k_timer_init(&acc_data_timer, acc_timer_handler, NULL);
    k_timer_start(&acc_data_timer, K_MSEC(0), K_MSEC(100));

    while (1)
    {
        k_sleep(K_FOREVER);
    }
    return 0; // always return something
}
