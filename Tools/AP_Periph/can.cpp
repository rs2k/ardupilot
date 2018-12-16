#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_Periph.h"
#include "hal.h"
#include "../../modules/libcanard/canard.h"

extern const AP_HAL::HAL &hal;

// simple puts() for debugging
static void puts(const char *str)
{
    chnWrite(&SD1, (const uint8_t *)str, strlen(str));
    chnWrite(&SD1, (const uint8_t *)"\n", 1);
}

/*
  simple printf for debugging
 */
static void printf(const char *fmt, ...)
{
    va_list arg_list;
    va_start(arg_list, fmt);
    char buf[60];
    int n = hal.util->vsnprintf(buf, sizeof(buf), fmt, arg_list);
    va_end(arg_list);
    if (n > 0) {
        chnWrite(&SD1, (const uint8_t *)buf, n);
    }
}

static CanardInstance canard;
static uint32_t canard_memory_pool[1024/4];
static const uint8_t PreferredNodeID = CANARD_BROADCAST_NODE_ID;

// can config for 1MBit
static uint32_t baudrate = 1000000U;

static const CANConfig cancfg = {
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    CAN_BTR_SJW(0) | CAN_BTR_TS2(2-1) |
    CAN_BTR_TS1(15-1) | CAN_BTR_BRP((STM32_PCLK1/18)/baudrate - 1)
};

#define APP_VERSION_MAJOR                                           1
#define APP_VERSION_MINOR                                           0
#define APP_NODE_NAME                                               "org.uavcan.libcanard.ap_periph"

/*
 * Some useful constants defined by the UAVCAN specification.
 * Data type signature values can be easily obtained with the script show_data_type_info.py
 */
#define UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_ID                      1
#define UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_SIGNATURE               0x0b2a812620a11d40
#define UAVCAN_NODE_ID_ALLOCATION_RANDOM_TIMEOUT_RANGE_USEC         400000UL
#define UAVCAN_NODE_ID_ALLOCATION_REQUEST_DELAY_OFFSET_USEC         600000UL

#define UAVCAN_NODE_STATUS_MESSAGE_SIZE                             7
#define UAVCAN_NODE_STATUS_DATA_TYPE_ID                             341
#define UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE                      0x0f0868d0c1a7c6f1

#define UAVCAN_NODE_HEALTH_OK                                       0
#define UAVCAN_NODE_HEALTH_WARNING                                  1
#define UAVCAN_NODE_HEALTH_ERROR                                    2
#define UAVCAN_NODE_HEALTH_CRITICAL                                 3

#define UAVCAN_NODE_MODE_OPERATIONAL                                0
#define UAVCAN_NODE_MODE_INITIALIZATION                             1

#define UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE                      ((3015 + 7) / 8)
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE                    0xee468a8121c46a9e
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_ID                           1

#define UAVCAN_BEGIN_FIRMWARE_UPDATE_DATA_TYPE_ID                   40
#define UAVCAN_BEGIN_FIRMWARE_UPDATE_DATA_TYPE_SIGNATURE            0xb7d725df72724126
#define UAVCAN_BEGIN_FIRMWARE_UPDATE_RESPONSE_MAX_SIZE              ((1031+7)/8)
#define UAVCAN_BEGIN_FIRMWARE_UPDATE_REQUEST_MAX_SIZE               ((1616+7)/8)

#define UAVCAN_RESTART_NODE_DATA_TYPE_ID                            5
#define UAVCAN_RESTART_NODE_DATA_TYPE_SIGNATURE                     0x569e05394a3017f0
#define UAVCAN_RESTART_NODE_REQUEST_MAX_SIZE                        ((40+7)/8)
#define UAVCAN_RESTART_NODE_RESPONSE_MAX_SIZE                       ((1+7)/8)

#define UNIQUE_ID_LENGTH_BYTES                                      16

/*
 * Variables used for dynamic node ID allocation.
 * RTFM at http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
 */
static uint64_t send_next_node_id_allocation_request_at;    ///< When the next node ID allocation request should be sent
static uint8_t node_id_allocation_unique_id_offset;         ///< Depends on the stage of the next request

/*
 * Node status variables
 */
static uint8_t node_health = UAVCAN_NODE_HEALTH_OK;
static uint8_t node_mode   = UAVCAN_NODE_MODE_INITIALIZATION;


static uint64_t getMonotonicTimestampUSec(void)
{
    return AP_HAL::micros64();
}


/**
 * Returns a pseudo random float in the range [0, 1].
 */
static float getRandomFloat(void)
{
    return float(get_random16()) / 0xFFFF;
}


/*
  get cpu unique ID
 */
static void readUniqueID(uint8_t* out_uid)
{
    uint8_t len = UNIQUE_ID_LENGTH_BYTES;
    memset(out_uid, 0, len);
    hal.util->get_system_id_unformatted(out_uid, len);
}


static void makeNodeStatusMessage(uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE])
{
    memset(buffer, 0, UAVCAN_NODE_STATUS_MESSAGE_SIZE);

    static uint32_t started_at_sec = 0;
    if (started_at_sec == 0)
    {
        started_at_sec = (uint32_t)(getMonotonicTimestampUSec() / 1000000U);
    }

    const uint32_t uptime_sec = (uint32_t)((getMonotonicTimestampUSec() / 1000000U) - started_at_sec);

    /*
     * Here we're using the helper for demonstrational purposes; in this simple case it could be preferred to
     * encode the values manually.
     */
    canardEncodeScalar(buffer,  0, 32, &uptime_sec);
    canardEncodeScalar(buffer, 32,  2, &node_health);
    canardEncodeScalar(buffer, 34,  3, &node_mode);
}

/*
  handle a GET_NODE_INFO request
 */
static void handle_get_node_info(CanardInstance* ins,
                                 CanardRxTransfer* transfer)
{
    uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE] {};

    // NodeStatus
    makeNodeStatusMessage(buffer);

    // SoftwareVersion
    buffer[7] = APP_VERSION_MAJOR;
    buffer[8] = APP_VERSION_MINOR;
    buffer[9] = 1;                          // Optional field flags, VCS commit is set
    uint32_t u32 = 1234; // XXXX git hash
    canardEncodeScalar(buffer, 80, 32, &u32);
    // Image CRC skipped

    // HardwareVersion
    // Major skipped
    // Minor skipped
    readUniqueID(&buffer[24]);
    // Certificate of authenticity skipped

    // Name
    const size_t name_len = strlen(APP_NODE_NAME);
    memcpy(&buffer[41], APP_NODE_NAME, name_len);

    const size_t total_size = 41 + name_len;

    /*
     * Transmitting; in this case we don't have to release the payload because it's empty anyway.
     */
    const int16_t resp_res = canardRequestOrRespond(ins,
                                                    transfer->source_node_id,
                                                    UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE,
                                                    UAVCAN_GET_NODE_INFO_DATA_TYPE_ID,
                                                    &transfer->transfer_id,
                                                    transfer->priority,
                                                    CanardResponse,
                                                    &buffer[0],
                                                    (uint16_t)total_size);
    if (resp_res <= 0) {
        puts("Could not respond to GetNodeInfo");
    }
}


static void handle_begin_firmware_update(CanardInstance* ins, CanardRxTransfer* transfer)
{
    // instant reboot for now
    NVIC_SystemReset();
}


/**
 * This callback is invoked by the library when a new message or request or response is received.
 */
static void onTransferReceived(CanardInstance* ins,
                               CanardRxTransfer* transfer)
{
    /*
     * Dynamic node ID allocation protocol.
     * Taking this branch only if we don't have a node ID, ignoring otherwise.
     */
    if ((canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID) &&
        (transfer->transfer_type == CanardTransferTypeBroadcast) &&
        (transfer->data_type_id == UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_ID))
    {
        // Rule C - updating the randomized time interval
        send_next_node_id_allocation_request_at =
            getMonotonicTimestampUSec() + UAVCAN_NODE_ID_ALLOCATION_REQUEST_DELAY_OFFSET_USEC +
            (uint64_t)(getRandomFloat() * UAVCAN_NODE_ID_ALLOCATION_RANDOM_TIMEOUT_RANGE_USEC);

        if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID)
        {
            puts("Allocation request from another allocatee");
            node_id_allocation_unique_id_offset = 0;
            return;
        }

        // Copying the unique ID from the message
        static const uint8_t UniqueIDBitOffset = 8;
        uint8_t received_unique_id[UNIQUE_ID_LENGTH_BYTES];
        uint8_t received_unique_id_len = 0;
        for (; received_unique_id_len < (transfer->payload_len - (UniqueIDBitOffset / 8U)); received_unique_id_len++)
        {
            assert(received_unique_id_len < UNIQUE_ID_LENGTH_BYTES);
            const uint8_t bit_offset = (uint8_t)(UniqueIDBitOffset + received_unique_id_len * 8U);
            (void) canardDecodeScalar(transfer, bit_offset, 8, false, &received_unique_id[received_unique_id_len]);
        }

        // Obtaining the local unique ID
        uint8_t my_unique_id[UNIQUE_ID_LENGTH_BYTES];
        readUniqueID(my_unique_id);

        // Matching the received UID against the local one
        if (memcmp(received_unique_id, my_unique_id, received_unique_id_len) != 0)
        {
            puts("Mismatching allocation response");
            node_id_allocation_unique_id_offset = 0;
            return;         // No match, return
        }

        if (received_unique_id_len < UNIQUE_ID_LENGTH_BYTES)
        {
            // The allocator has confirmed part of unique ID, switching to the next stage and updating the timeout.
            node_id_allocation_unique_id_offset = received_unique_id_len;
            send_next_node_id_allocation_request_at -= UAVCAN_NODE_ID_ALLOCATION_REQUEST_DELAY_OFFSET_USEC;

            puts("Matching allocation response");
        }
        else
        {
            // Allocation complete - copying the allocated node ID from the message
            uint8_t allocated_node_id = 0;
            (void) canardDecodeScalar(transfer, 0, 7, false, &allocated_node_id);
            assert(allocated_node_id <= 127);

            canardSetLocalNodeID(ins, allocated_node_id);
            puts("Node ID allocated");
        }
    }

    if (transfer->transfer_type != CanardTransferTypeRequest) {
        return;
    }
    switch (transfer->data_type_id) {
    case UAVCAN_GET_NODE_INFO_DATA_TYPE_ID:
        handle_get_node_info(ins, transfer);
        break;

    case UAVCAN_BEGIN_FIRMWARE_UPDATE_DATA_TYPE_ID:
        handle_begin_firmware_update(ins, transfer);
        break;
    }
}


/**
 * This callback is invoked by the library when it detects beginning of a new transfer on the bus that can be received
 * by the local node.
 * If the callback returns true, the library will receive the transfer.
 * If the callback returns false, the library will ignore the transfer.
 * All transfers that are addressed to other nodes are always ignored.
 */
static bool shouldAcceptTransfer(const CanardInstance* ins,
                                 uint64_t* out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
    (void)source_node_id;

    if (canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID)
    {
        /*
         * If we're in the process of allocation of dynamic node ID, accept only relevant transfers.
         */
        if ((transfer_type == CanardTransferTypeBroadcast) &&
            (data_type_id == UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_ID))
        {
            *out_data_type_signature = UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_SIGNATURE;
            return true;
        }
        return false;
    }
    if (transfer_type != CanardTransferTypeRequest) {
        return false;
    }
    switch (data_type_id) {
    case UAVCAN_GET_NODE_INFO_DATA_TYPE_ID:
        *out_data_type_signature = UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE;
        return true;
    case UAVCAN_BEGIN_FIRMWARE_UPDATE_DATA_TYPE_ID:
        *out_data_type_signature = UAVCAN_BEGIN_FIRMWARE_UPDATE_DATA_TYPE_SIGNATURE;
        return true;
    case UAVCAN_RESTART_NODE_DATA_TYPE_ID:
        *out_data_type_signature = UAVCAN_RESTART_NODE_DATA_TYPE_SIGNATURE;
        return true;
    default:
        break;
    }

    return false;
}

static void processTx(void)
{
    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
        CANTxFrame txmsg {};
        txmsg.DLC = txf->data_len;
        memcpy(txmsg.data8, txf->data, 8);
        txmsg.EID = txf->id & CANARD_CAN_EXT_ID_MASK;
        txmsg.IDE = 1;
        txmsg.RTR = 0;
        if (canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_IMMEDIATE) == MSG_OK) {
            canardPopTxQueue(&canard);
        } else {
            // Timeout - just exit and try again later
            return;
        }
    }
}

CANRxFrame *last_rxmsg;

static void processRx(void)
{
    CANRxFrame rxmsg {};
    last_rxmsg = &rxmsg;
    while (canReceive(&CAND1, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE) == MSG_OK) {
        CanardCANFrame rx_frame {};

        palToggleLine(HAL_GPIO_PIN_LED);

        const uint64_t timestamp = AP_HAL::micros64();
        memcpy(rx_frame.data, rxmsg.data8, 8);
        rx_frame.data_len = rxmsg.DLC;
        if(rxmsg.IDE) {
            rx_frame.id = CANARD_CAN_FRAME_EFF | rxmsg.EID;
        } else {
            rx_frame.id = rxmsg.SID;
        }
        canardHandleRxFrame(&canard, &rx_frame, timestamp);
    }
}

/**
 * This function is called at 1 Hz rate from the main loop.
 */
static void process1HzTasks(uint64_t timestamp_usec)
{
    /*
     * Purging transfers that are no longer transmitted. This will occasionally free up some memory.
     */
    canardCleanupStaleTransfers(&canard, timestamp_usec);

    /*
     * Printing the memory usage statistics.
     */
    {
        const CanardPoolAllocatorStatistics stats = canardGetPoolAllocatorStatistics(&canard);
        const uint16_t peak_percent = (uint16_t)(100U * stats.peak_usage_blocks / stats.capacity_blocks);

        /*
         * The recommended way to establish the minimal size of the memory pool is to stress-test the application and
         * record the worst case memory usage.
         */
        if (peak_percent > 70)
        {
            puts("WARNING: ENLARGE MEMORY POOL");
        }
    }

    /*
     * Transmitting the node status message periodically.
     */
    {
        uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE];
        makeNodeStatusMessage(buffer);


        static uint8_t transfer_id;  // Note that the transfer ID variable MUST BE STATIC (or heap-allocated)!

        const int16_t bc_res = canardBroadcast(&canard,
                                               UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE,
                                               UAVCAN_NODE_STATUS_DATA_TYPE_ID,
                                               &transfer_id,
                                               CANARD_TRANSFER_PRIORITY_LOW,
                                               buffer,
                                               UAVCAN_NODE_STATUS_MESSAGE_SIZE);
        if (bc_res <= 0)
        {
            printf("broadcast fail %d\n", bc_res);
        } else {
            puts("broadcast node status OK");
        }
    }

    node_mode = UAVCAN_NODE_MODE_OPERATIONAL;
}

/*
  wait for dynamic allocation of node ID
 */
static void can_wait_node_id(void)
{
    uint8_t node_id_allocation_transfer_id = 0;

    while (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID)
    {
        puts("Waiting for dynamic node ID allocation...");

        send_next_node_id_allocation_request_at =
            getMonotonicTimestampUSec() + UAVCAN_NODE_ID_ALLOCATION_REQUEST_DELAY_OFFSET_USEC +
            (uint64_t)(getRandomFloat() * UAVCAN_NODE_ID_ALLOCATION_RANDOM_TIMEOUT_RANGE_USEC);

        while ((getMonotonicTimestampUSec() < send_next_node_id_allocation_request_at) &&
               (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID))
        {
            processTx();
            processRx();
            canardCleanupStaleTransfers(&canard, AP_HAL::micros());
        }

        if (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID)
        {
            break;
        }

        // Structure of the request is documented in the DSDL definition
        // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
        uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
        allocation_request[0] = (uint8_t)(PreferredNodeID << 1U);

        if (node_id_allocation_unique_id_offset == 0)
        {
            allocation_request[0] |= 1;     // First part of unique ID
        }

        uint8_t my_unique_id[UNIQUE_ID_LENGTH_BYTES];
        readUniqueID(my_unique_id);

        static const uint8_t MaxLenOfUniqueIDInRequest = 6;
        uint8_t uid_size = (uint8_t)(UNIQUE_ID_LENGTH_BYTES - node_id_allocation_unique_id_offset);
        if (uid_size > MaxLenOfUniqueIDInRequest)
        {
            uid_size = MaxLenOfUniqueIDInRequest;
        }

        // Paranoia time
        assert(node_id_allocation_unique_id_offset < UNIQUE_ID_LENGTH_BYTES);
        assert(uid_size <= MaxLenOfUniqueIDInRequest);
        assert(uid_size > 0);
        assert((uid_size + node_id_allocation_unique_id_offset) <= UNIQUE_ID_LENGTH_BYTES);

        memmove(&allocation_request[1], &my_unique_id[node_id_allocation_unique_id_offset], uid_size);

        // Broadcasting the request
        const int16_t bcast_res = canardBroadcast(&canard,
                                                  UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_SIGNATURE,
                                                  UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_ID,
                                                  &node_id_allocation_transfer_id,
                                                  CANARD_TRANSFER_PRIORITY_LOW,
                                                  &allocation_request[0],
                                                  (uint16_t) (uid_size + 1));
        if (bcast_res < 0)
        {
            printf("Could not broadcast ID allocation req; error %d\n", bcast_res);
        }

        // Preparing for timeout; if response is received, this value will be updated from the callback.
        node_id_allocation_unique_id_offset = 0;
    }

    printf("Dynamic node ID allocation complete [%d]\n", canardGetLocalNodeID(&canard));
}

void AP_Periph_FW::can_start()
{
    canStart(&CAND1, &cancfg);

    canardInit(&canard, (uint8_t *)canard_memory_pool, sizeof(canard_memory_pool),
               onTransferReceived, shouldAcceptTransfer, NULL);

    // wait for dynamic node ID allocation
    can_wait_node_id();
}


void AP_Periph_FW::can_update()
{
    static uint32_t last_1Hz_ms;
    uint32_t now = AP_HAL::millis();
    if (now - last_1Hz_ms >= 1000) {
        last_1Hz_ms = now;
        process1HzTasks(now * 1000UL);
    }
    processTx();
    processRx();
}
