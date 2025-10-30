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
#include <zephyr/bluetooth/hci.h>
#include <zephyr/net/buf.h>
#include <zephyr/pm/pm.h>
#include <hal/nrf_gpio.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/* Define your trigger signatures */
static const uint8_t trigger_signature[] = {0x59, 0x00, 0xBB, 0xCC}; // Normal transmission trigger
static const uint8_t reset_signature[] = {0x59, 0x00, 0xFF, 0xFF};   // Special reset command

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
#define RSSI_THRESHOLD -50

static uint8_t tx_buffer[TX_BUFFER_SIZE] = {0}; // 0..255 → 256 bytes
uint8_t dummy_buf[TX_VALID_SIZE] = {0};         // All zeros
static uint8_t mfg_data[TX_VALID_SIZE] = {0};   // Match size exactly to avoid overflow
static uint8_t tx_index = 0;
static uint8_t packet_id = 1;

/* BLE / Advertising Variables */
static struct bt_le_ext_adv *adv;
static struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
    BT_LE_ADV_OPT_NONE | BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_USE_IDENTITY | BT_LE_ADV_OPT_NO_2M,
    0x30, 0x30, NULL);

static struct bt_le_ext_adv_start_param ext_adv_param = BT_LE_EXT_ADV_START_PARAM_INIT(0, 2);
static const struct bt_data ap[] = {
    BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)), // Uses 256 bytes safely
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};
static const struct bt_data ad[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* LED Config */
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

/* Sensor Config */
static const struct device *sensor = DEVICE_DT_GET_ANY(st_lis2dh);
#define nodeId CONFIG_NODE_ID
#define sensorAddress CONFIG_SENSOR_ADDRESS

/* Work & Timer */
static struct k_work adver_work;
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
 * BLE Scanning Support
 * ============================ */
/* Parser context */
struct adv_parse_context
{
        int8_t rssi;
        bool trigger_found;
        bool reset_found;
};

/* Parser callback for advertising data */
static bool ad_parser_cb(struct bt_data *data, void *user_data)
{
        struct adv_parse_context *ctx = (struct adv_parse_context *)user_data;

        if (data->type == BT_DATA_MANUFACTURER_DATA)
        {
                /* Check for normal trigger */
                if (data->data_len >= sizeof(trigger_signature) &&
                    memcmp(data->data, trigger_signature, sizeof(trigger_signature)) == 0)
                {
                        ctx->trigger_found = true;
                        return false; /* Stop parsing more AD fields */
                }

                /* Check for reset command */
                if (data->data_len >= sizeof(reset_signature) &&
                    memcmp(data->data, reset_signature, sizeof(reset_signature)) == 0)
                {
                        ctx->reset_found = true;
                        return false; /* Stop parsing as well */
                }
        }
        return true;
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi,
                    uint8_t type, struct net_buf_simple *ad)
{
        /* --- 1) Early RSSI filter --- */
        if (rssi < RSSI_THRESHOLD)
        {
                return;
        }

        /* --- 2) Prepare parser context --- */
        struct adv_parse_context ctx = {
            .rssi = rssi,
            .trigger_found = false,
            .reset_found = false,
        };

        /* Parse advertisement */
        bt_data_parse(ad, ad_parser_cb, &ctx);

        /* --- 3) Take action based on what was found --- */
        if (ctx.trigger_found)
        {
                char addr_str[BT_ADDR_LE_STR_LEN];
                bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
                printk("[SCAN] Trigger matched from %s RSSI=%d\n", addr_str, rssi);

                /* Start transmission work */
                k_work_submit(&adver_work);
        }
        else if (ctx.reset_found)
        {
                char addr_str[BT_ADDR_LE_STR_LEN];
                bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
                printk("[SCAN] Reset command received from %s RSSI=%d\n", addr_str, rssi);

                /* Call reset function directly */
                reset_state();
        }
}

/* --- Scan parameters --- */
static struct bt_le_scan_param scan_param = {
    .type = BT_LE_SCAN_TYPE_PASSIVE,
    .options = BT_LE_SCAN_OPT_NONE,
    .interval = 0x0030, // 30 ms
    .window = 0x0030,
};

/* --- Start scanning --- */
void start_scanning()
{
        int err = bt_le_scan_start(&scan_param, scan_cb);
        if (err)
        {
                printk("Failed to start scanning (err %d)\n", err);
        }
        else
        {
                printk("Scanning started...\n");
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

void transmit_data(void)
{
        /* Stop scanning first */
        bt_le_scan_stop();

        int err = bt_le_ext_adv_set_data(adv, ap, ARRAY_SIZE(ap), NULL, 0);
        if (err)
        {
                printk("Failed to set adv data (err %d)\n", err);
                start_scanning();
                return;
        }

        printk("Start Extended Advertising...\n");
        err = bt_le_ext_adv_start(adv, &ext_adv_param);
        if (err)
        {
                printk("Failed to start advertising (err %d)\n", err);
                start_scanning();
                return;
        }

        k_msleep(500); // keep advertising briefly

        bt_le_ext_adv_stop(adv);
        printk("Stopped advertising.\n");

        start_scanning(); // restart scanning after adv stop
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
        k_msleep(50);
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
                        tx_buffer[TX_HEADER_SIZE + TX_PAYLOAD_SIZE + 2] = 0xFE;        // Footer

                        write_to_flash(tx_buffer, TX_PACKET_SIZE); // Write full 256 bytes

                        tx_index = 0; // Reset for next packet
                }
        }
}

void reset_state(void)
{
        writeIndex = readIndex = file_count = package_count = packet_id = tx_index = 0;
        write_storage[0] = writeIndex;
        write_storage[1] = file_count;

        const struct device *flashDev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
        if (!device_is_ready(flashDev))
        {
                printk("Flash device not ready\n");
                return;
        }

        k_mutex_lock(&testMutex, K_FOREVER);
        gpio_pin_set_dt(&led1, 1);
        flash_erase(flashDev, save_location, SPI_FLASH_SECTOR_SIZE);
        flash_write(flashDev, save_location, write_storage, sizeof(write_storage));
        gpio_pin_set_dt(&led1, 0);
        k_mutex_unlock(&testMutex);
        printk("Data reset done\n");
        k_msleep(100);
}

// void adver(void)
// {
//         while (1)
//         {
//                 const struct device *flashDev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
//                 if (!device_is_ready(flashDev))
//                         return;

//                 task2_active = true;
//                 uint32_t pending = (writeIndex >= readIndex) ? (writeIndex - readIndex) : (flashLimit - readIndex + writeIndex);

//                 if (pending == 0)
//                 {
//                         printk("No data to send, starting new fetch\n");
//                         task2_active = false;
//                         k_work_submit(&acc_data_work);
//                         return;
//                 }

//                 if (readIndex >= flashLimit)
//                         readIndex = 0;

//                 while (pending >= TX_PACKET_SIZE)
//                 {
//                         uint32_t chunk = (pending >= TX_PACKET_SIZE) ? TX_PACKET_SIZE : pending;

//                         /* Read only header+payload+footer from flash */
//                         if (flash_read(flashDev, readIndex, tx_buffer, TX_VALID_SIZE))
//                                 break;

//                         for (int pkt = 0; pkt < 5; pkt++) // Send 2 dummy packets
//                         {
//                                 memset(dummy_buf, 0x00, TX_VALID_SIZE);

//                                 dummy_buf[0] = 0x59;                // Header byte 1
//                                 dummy_buf[1] = 0x00;                // Header byte 2
//                                 dummy_buf[TX_VALID_SIZE - 3] = pkt; // Dummy Packet ID
//                                 dummy_buf[TX_VALID_SIZE - 2] = 0x00;
//                                 dummy_buf[TX_VALID_SIZE - 1] = 0xFE;

//                                 memcpy(mfg_data, dummy_buf, TX_VALID_SIZE);
//                                 transmit_name();
//                         }

//                         /* -------- 2) Now Transmit Actual Data -------- */
//                         memcpy(mfg_data, tx_buffer, TX_VALID_SIZE);

//                         printk("Read %d bytes from flash [0x%08X → 0x%08X]\n",
//                                chunk, (unsigned int)readIndex, (unsigned int)(readIndex + chunk - 1));

//                         // for (int i = 0; i < TX_VALID_SIZE; i++)
//                         // {
//                         //         printk("%02X ", tx_buffer[i]);
//                         //         if ((i + 1) % 16 == 0)
//                         //                 printk("\n");
//                         // }
//                         // printk("\n");

//                         gpio_pin_set_dt(&led0, 1);
//                         transmit_data();
//                         gpio_pin_set_dt(&led0, 0);

//                         /* Move to next packet in flash */
//                         readIndex = (readIndex + chunk) % flashLimit;
//                         pending -= chunk;
//                         save_state(flashDev);
//                 }
//                 task2_active = false;
//                 // }
//                 k_msleep(500);
//                 break;
//         }
// }

void adver(void)
{
        static int64_t last_tx_time = 0; // Keeps track of last transmission time
        int64_t now = k_uptime_get();
        bool send_name = false;

        // Decide if we should send name (dummy packets)
        if ((now - last_tx_time) > 5000) // If more than 5 sec gap
        {
                send_name = true;
        }

        while (1)
        {
                const struct device *flashDev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
                if (!device_is_ready(flashDev))
                        return;

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
                                for (int pkt = 0; pkt < 5; pkt++)
                                {
                                        memset(dummy_buf, 0x00, TX_VALID_SIZE);

                                        dummy_buf[0] = 0x59;                // Header byte 1
                                        dummy_buf[1] = 0x00;                // Header byte 2
                                        dummy_buf[TX_VALID_SIZE - 3] = pkt; // Dummy Packet ID
                                        dummy_buf[TX_VALID_SIZE - 2] = 0x00;
                                        dummy_buf[TX_VALID_SIZE - 1] = 0xFE;

                                        memcpy(mfg_data, dummy_buf, TX_VALID_SIZE);
                                        transmit_name();
                                }
                                send_name = false; // Done once
                        }

                        /* -------- 2) Now Transmit Actual Data -------- */
                        memcpy(mfg_data, tx_buffer, TX_VALID_SIZE);

                        printk("Read %d bytes from flash [0x%08X → 0x%08X]\n",
                               chunk, (unsigned int)readIndex, (unsigned int)(readIndex + chunk - 1));

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
                k_msleep(500);
                break;
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
        ;
        k_msleep(4000);
}

// int main(void)
// {
//         sensor_init();
//         printk("Starting Data Logger Node\n");

//         set_random_static_address();
//         if (bt_enable(NULL))
//         {
//                 printk("Bluetooth init failed");
//                 return -1;
//         }
//         k_msleep(100);
//         adv_param_init();
//          start_scanning();

//         const struct device *flashDev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
//         if (!device_is_ready(flashDev))
//         {
//                 printk("Flash device not ready");
//                 return -1;
//         }
//         restore_state(flashDev);

//         k_work_init(&adver_work, (k_work_handler_t)adver);
//         k_timer_init(&button_timer, button_timer_handler, NULL);
//         k_timer_start(&button_timer, K_MSEC(50), K_MSEC(50));

//         /* --- Accelerometer data timer: fires every 500ms --- */
//         k_work_init(&acc_data_work, (k_work_handler_t)fetch_acc_data);
//         k_timer_init(&acc_data_timer, acc_timer_handler, NULL);
//         k_timer_start(&acc_data_timer, K_MSEC(0), K_MSEC(500));
//         while (1)
//         {
//                 k_sleep(K_FOREVER);
//         }
//         return 0; // always return something
// }
int main(void)
{
        sensor_init();
        printk("Starting Data Logger Node\n");

        /* Initialize Bluetooth */
        if (bt_enable(NULL))
        {
                printk("Bluetooth init failed\n");
                return -1;
        }

        k_msleep(100);
        set_random_static_address();
        adv_param_init();

        /* Prepare Flash */
        const struct device *flashDev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
        if (!device_is_ready(flashDev))
        {
                printk("Flash device not ready\n");
                return -1;
        }
        restore_state(flashDev);

        /* Initialize work for advertising */
        k_work_init(&adver_work, (k_work_handler_t)adver);

        /* Initialize accelerometer work & timer */
        k_work_init(&acc_data_work, (k_work_handler_t)fetch_acc_data);
        k_timer_init(&acc_data_timer, acc_timer_handler, NULL);
        k_timer_start(&acc_data_timer, K_MSEC(0), K_MSEC(500));

        /* --- Start scanning automatically --- */
        start_scanning();

        /* Main thread sleeps forever */
        while (1)
        {
                k_sleep(K_FOREVER);
        }

        return 0;
}